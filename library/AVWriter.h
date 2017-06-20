/**
 * AVWriter
 *
 * Threaded object for creating and writing to a video file.
 * 
 * Richard Moore, 2009.
 * rjdmoore@uqconnect.edu.au
 */

#ifndef AVWRITER_H_
#define AVWRITER_H_

#define OCV_AVWRITER

#ifndef OCV_AVWRITER
extern "C" {
	#include <libavcodec/avcodec.h>
	#include <libavformat/avformat.h>
	#include <libswscale/swscale.h>
}
#include <boost/shared_array.hpp>
#endif // ndef OCV_AVWRITER

#include <opencv2/opencv.hpp>

#include <pthread.h>

#include <sys/time.h>

#include <string>
#include <stdint.h>

void* PROCESS_FRAMES_TO_WRITE(void* avwriter);

// copied from ffmpeg
enum AVPixelFormat {
    AV_PIX_FMT_RGB24,     ///< packed RGB 8:8:8, 24bpp, RGBRGB...
    AV_PIX_FMT_BGR24,     ///< packed RGB 8:8:8, 24bpp, BGRBGR...
    AV_PIX_FMT_GRAY8      ///<        Y        ,  8bpp
};

class AVWriter
{
	public:
		AVWriter(std::string filename,
				size_t src_width, size_t src_height,
				size_t dst_width, size_t dst_height,
				AVPixelFormat pix_fmt,
				int fps=25, int cpu=-1, int q=2);
		~AVWriter();
		void enqueue_frame(cv::Mat& frame);
		inline bool isActive() { return _active; }
		
	private:
		bool				_active;
		pthread_t			_thread;
		pthread_mutex_t		_mutex;
		pthread_cond_t		_cond;
		std::string			_filename;
		size_t				_srcWidth, _srcHeight, _dstWidth, _dstHeight;

		AVPixelFormat		_pixFmt;
		int					_cpu;

#ifndef OCV_AVWRITER
		AVFormatContext*	format_context;
		AVCodecContext*		codec_context;
		SwsContext*			sws_context;
		AVCodec*			codec;
		AVStream*			avstream;
//		AVFrame*			avframe_raw;
		AVFrame*			avframe_scl;
//		uint8_t*			frame_buffer_raw;
		uint8_t*			frame_buffer_scl;
		uint8_t*			video_buffer;
		int					video_buff_size;

		std::deque<boost::shared_array<uint8_t> >	_frames_to_write;

		int write_frame(boost::shared_array<uint8_t> frame);
#else
		cv::VideoWriter		_writer;
		std::deque<cv::Mat>	_frames_to_write;
#endif // ndef OCV_AVWRITER
		
//		std::string			_avtiming;
//		float				av_vid_time, av_scl_time, av_enc_time, av_wrt_time;
		
		friend void* PROCESS_FRAMES_TO_WRITE(void* avw);
};

#endif /*AVWRITER_H_*/

