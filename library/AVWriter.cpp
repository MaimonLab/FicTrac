/*
 * ACWriter.cpp
 *
 *  Created on: 14/03/2012
 *      Author: Richard Moore (rjdmoore@gmail.com)
 */

#include "AVWriter.h"
#include <Utils.h>

extern "C" {
	#include <libavutil/mathematics.h>
}

/// New FFmpeg breaks backwards compatibility - we should probably all just upgrade.
#ifndef CODEC_TYPE_VIDEO
	#define CODEC_TYPE_VIDEO AVMEDIA_TYPE_VIDEO
#endif
#ifndef PKT_FLAG_KEY
	#define PKT_FLAG_KEY AV_PKT_FLAG_KEY
#endif

//static const PixelFormat DST_FORMAT = PIX_FMT_YUVJ420P;
//static const PixelFormat DST_FORMAT = PIX_FMT_YUVJ422P;
static const PixelFormat DST_FORMAT = PIX_FMT_YUV420P;
//static const PixelFormat DST_FORMAT = PIX_FMT_YUV422P;

// maximum allowed length of video write queue
// extra frames will be dropped
static const unsigned int AV_MAX_VIDQ_SIZE = 100;

using std::string;
using std::max;

AVWriter::AVWriter(std::string filename,
		size_t src_width, size_t src_height,
		size_t dst_width, size_t dst_height,
		PixelFormat pix_fmt, int fps, int cpu, int q)
:	_active(true), _thread(0), _filename(filename),
	_srcWidth(src_width), _srcHeight(src_height),
	_dstWidth(dst_width), _dstHeight(dst_height),
	_pixFmt(pix_fmt), _cpu(cpu), sws_context(NULL)
{
	av_register_all();

#if (LIBAVFORMAT_VERSION_MAJOR >= 53)
	avformat_network_init();
#endif

	pthread_mutex_init(&_mutex,NULL);
	pthread_cond_init(&_cond,NULL);

	// allocate format context
	format_context = avformat_alloc_context();
	format_context->oformat = av_guess_format("avi",NULL,NULL);
	snprintf(format_context->filename, sizeof(format_context->filename), "%s.avi", _filename.c_str());

	// allocate stream
#if (LIBAVFORMAT_VERSION_MAJOR >= 54)
	avstream = avformat_new_stream(format_context, 0);
#else
	avstream = av_new_stream(format_context, 0);
#endif

	// allocate codec context
	codec_context = avstream->codec;
	codec_context->codec_type = CODEC_TYPE_VIDEO;
//	codec_context->codec_id = CODEC_ID_MJPEG;
	codec_context->codec_id = CODEC_ID_MPEG2VIDEO;
	codec_context->pix_fmt = DST_FORMAT;
	codec_context->width = _dstWidth;
	codec_context->height = _dstHeight;
	codec_context->time_base = (AVRational){1,fps};
//	codec_context->gop_size = 12;
//	codec_context->me_method = ME_EPZS; // or ME_FULL
//	codec_context->prediction_method = FF_PRED_LEFT; //or FF_PRED_PLANE, FF_PRED_MEDIAN
//	codec_context->max_b_frames = 0;
//	codec_context->bits_per_sample = 8;
	codec_context->qmin = q;
	codec_context->qmax = q;

#if (LIBAVFORMAT_VERSION_MAJOR >= 54)
	avformat_write_header(format_context,NULL);
#else
	av_set_parameters(format_context,NULL);
#endif

	// open codec
	codec = avcodec_find_encoder(codec_context->codec_id);
#if (LIBAVFORMAT_VERSION_MAJOR >= 54)
	avcodec_open2(codec_context,codec,NULL);
#else
	avcodec_open(codec_context,codec);
#endif

	// allocate frame
//    avframe_raw = avcodec_alloc_frame();
    avframe_scl = avcodec_alloc_frame();

    // allocate data
//    int frame_size_raw = avpicture_get_size(_pixFmt, _srcWidth, _srcHeight);
//    frame_buffer_raw = (uint8_t*)av_malloc(frame_size_raw*sizeof(uint8_t));
//    avpicture_fill((AVPicture*)avframe_raw, frame_buffer_raw, _pixFmt, _srcWidth, _srcHeight);

    int frame_size_scl = avpicture_get_size(codec_context->pix_fmt,
    		codec_context->width, codec_context->height);
    frame_buffer_scl = (uint8_t*)av_malloc(frame_size_scl);
    avpicture_fill((AVPicture*)avframe_scl, frame_buffer_scl,
    		codec_context->pix_fmt, codec_context->width, codec_context->height);

	// allocate video buffer
	video_buff_size = max(frame_size_scl + 4096, 10*FF_MIN_BUFFER_SIZE);
	video_buffer = (uint8_t*)av_malloc(video_buff_size*sizeof(uint8_t));

    // allocate file if neccessary
	if( !(format_context->oformat->flags & AVFMT_NOFILE) ) {
#if (LIBAVFORMAT_VERSION_MAJOR >= 54)
		if( avio_open(&format_context->pb, format_context->filename, AVIO_FLAG_WRITE) < 0 ) {
#else
        if( url_fopen(&format_context->pb, format_context->filename, URL_WRONLY) < 0 ) {
#endif
            printf("%s: Error, could not open '%s'\n", __func__, format_context->filename);
            _active = false;
            av_freep(&format_context);
            return;
        }
    }

    // write video header
#if (LIBAVFORMAT_VERSION_MAJOR >= 54)
	avformat_write_header(format_context,NULL);
#else
    av_write_header(format_context);
#endif

    // allocate conversion context
    int flags = SWS_FAST_BILINEAR | SWS_PRINT_INFO;
    if( (codec_context->width % 32) == 0 ) {
    	flags |= (SWS_CPU_CAPS_MMX | SWS_CPU_CAPS_MMX2);
    }
    sws_context = sws_getContext(
    		_srcWidth, _srcHeight, _pixFmt,
    		codec_context->width, codec_context->height, codec_context->pix_fmt,
			flags, NULL, NULL, NULL);

    printf("%s: \t%s opened for video writing\n", __func__, format_context->filename);

	av_vid_time = 0;
	av_scl_time = 0;
	av_enc_time = 0;
	av_wrt_time = 0;

    // start thread
	pthread_create(&_thread, NULL, &PROCESS_FRAMES_TO_WRITE, this);
}

AVWriter::~AVWriter()
{
	// notify thread of stop condition
	_active = false;
	pthread_cond_broadcast(&_cond);

	printf("%s: Processing frames remaining in write queue.\n", __func__);

	// wait for writing to finish
	if( _thread ) {
		pthread_join(_thread,0);
	}

	pthread_cond_destroy(&_cond);
	pthread_mutex_destroy(&_mutex);

	_frames_to_write.clear();

	// close writer
	if( format_context ) {
		printf("%s: Closing video.. ", __func__);
		int err = av_write_trailer(format_context);
		if( err ) {
			printf("FAIL!\n");
		} else {
			printf("DONE.\n");
		}
	}

	sws_freeContext(sws_context);

//	av_freep(frame_buffer_raw);
//	av_free(avframe_raw);
	av_freep(&frame_buffer_scl);
	av_freep(&avframe_scl);
	av_freep(&video_buffer);

	avcodec_close(codec_context);

	if( format_context ) {
		for( unsigned int i = 0; i < format_context->nb_streams; ++i ) {
			av_freep(&format_context->streams[i]->codec);
			av_freep(&format_context->streams[i]);
		}
		if( !(format_context->oformat->flags & AVFMT_NOFILE) ) {
#if (LIBAVFORMAT_VERSION_MAJOR >= 54)
			avio_close(format_context->pb);
#else
			url_fclose(format_context->pb);
#endif
		}
		av_freep(&format_context);
	}
}

void* PROCESS_FRAMES_TO_WRITE(void* avw)
{
	Utils::SET_PROCESS_PRIORITY(-1);

	/// Get handle for this avwriter.
	AVWriter* avwriter = (AVWriter*)avw;

	/// Obtain lock.
	pthread_mutex_lock(&(avwriter->_mutex));

	/// Run while active.
	while( avwriter->_active ) {
		/// Release lock and wait for frames.
		while( avwriter->_active && avwriter->_frames_to_write.empty() ) {
			pthread_cond_wait(&(avwriter->_cond),&(avwriter->_mutex));
		}
		// write all frames (even if not active)
		while( !avwriter->_frames_to_write.empty() ) {
			/// Retrieve pointer to frame.
			boost::shared_array<uint8_t> frame = avwriter->_frames_to_write.front();

			/// Release lock.
			pthread_mutex_unlock(&(avwriter->_mutex));

			/// Write frame.
			int ret;
			if( (ret = avwriter->write_frame(frame)) < 0 ) {
				printf("%s: Error writing video frame (err %d)!\n", __func__, ret);
			}

			/// Obtain lock and release frame.
			pthread_mutex_lock(&(avwriter->_mutex));
			avwriter->_frames_to_write.pop_front();
		}
		/// If not active then exit.
	}

	/// Release lock.
	pthread_mutex_unlock(&(avwriter->_mutex));
	return NULL;
}

int AVWriter::write_frame(boost::shared_array<uint8_t> frame)
{
	timeval scl_time, enc_time, wrt_time, end_time;
	int w = _srcWidth;
	if( _pixFmt == PIX_FMT_RGB24 ||
		_pixFmt == PIX_FMT_BGR24 ) { w *= 3; }

	uint8_t* tmp_dat = frame.get();
//	// copy unaligned rows from src image to aligned(?) sws image
//	for( unsigned int i = 0; i < _srcHeight; ++i ) {
//		memcpy(avframe_raw->data[0]+i*avframe_raw->linesize[0],
//				tmp_dat+i*w, w);
//	}

	// FFMPEG hack
//	const uint8_t* avframedata[4] = {	avframe_raw->data[0],
//										avframe_raw->data[1],
//										avframe_raw->data[2],
//										avframe_raw->data[3]};
//	gettimeofday(&scl_time,NULL);

	// scale image to video format
	sws_scale(sws_context, &tmp_dat, &w, 0, _srcHeight,
			avframe_scl->data, avframe_scl->linesize);

//	gettimeofday(&enc_time,NULL);

	// compress frame
	int ret = 0;
	int len = avcodec_encode_video(codec_context,
			video_buffer, video_buff_size, avframe_scl);
	if( len > 0 ) {
		AVPacket pkt;
		av_init_packet(&pkt);

		if( codec_context->coded_frame->pts != (int64_t)AV_NOPTS_VALUE ) {
			pkt.pts = av_rescale_q(codec_context->coded_frame->pts,
					codec_context->time_base, avstream->time_base);
		}
		if( codec_context->coded_frame->key_frame ) {
			pkt.flags |= PKT_FLAG_KEY;
		}
		pkt.stream_index = avstream->index;
		pkt.data = video_buffer;
		pkt.size = len;

		// write compressed frame to media file
//		gettimeofday(&wrt_time, NULL);
		ret = av_write_frame(format_context, &pkt);

		// FFMPEG file syncing hack.
//		fdatasync(url_get_file_handle(url_fileno(format_context->pb)));
	} else {
		// frame buffered
//		gettimeofday(&wrt_time, NULL);
		ret = 0;

		// put special case flushing buffer here
	}
//	gettimeofday(&end_time,NULL);
//	av_vid_time = 1000.0*(end_time.tv_sec-scl_time.tv_sec)+(end_time.tv_usec-scl_time.tv_usec)/1000.0;
//	av_scl_time = 1000.0*(enc_time.tv_sec-scl_time.tv_sec)+(enc_time.tv_usec-scl_time.tv_usec)/1000.0;
//	av_enc_time = 1000.0*(wrt_time.tv_sec-enc_time.tv_sec)+(wrt_time.tv_usec-enc_time.tv_usec)/1000.0;
//	av_wrt_time = 1000.0*(end_time.tv_sec-wrt_time.tv_sec)+(end_time.tv_usec-wrt_time.tv_usec)/1000.0;
//	printf("%s: \tscale/encode/write times: %5.2f/%5.2f/%5.2f ms\n", __func__, av_scl_time, av_enc_time, av_wrt_time);
	return ret;
}

void AVWriter::enqueue_frame(cv::Mat& frame)
{
	if( !_active ) {
		printf("%s: Unable to add frames to queue, AVWriter is inactive!\n", __func__);
		return;
	}

	// obtain lock
	pthread_mutex_lock(&_mutex);

	if (_frames_to_write.size() > AV_MAX_VIDQ_SIZE) {
		// buffer too long
		printf("%s: Unable to add frames to queue, max length exceeded (%d)!\n",
				__func__, _frames_to_write.size());
	} else {
		// add frame to buffer
		int step = frame.cols*frame.channels();
		boost::shared_array<uint8_t> my_frame = boost::shared_array<uint8_t>(new uint8_t[frame.rows*step]);
		for( int i = 0, ii = 0; i < frame.rows; i++, ii+=step ) {
			memcpy(&my_frame[ii], frame.ptr(i), step);
		}
		_frames_to_write.push_back(my_frame);
//		printf("%s: \tthere are now %d frames in the write queue\n", __func__, _frames_to_write.size());
	}

	// notify
	pthread_cond_broadcast(&_cond);

	// release lock
	pthread_mutex_unlock(&_mutex);
}
