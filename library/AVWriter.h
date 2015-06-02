/**
 * AVWriter
 *
 * Threaded object for creating and writing to a video file.
 * 
 * Richard Moore, 2009.
 * rjdmoore@gmail.com
 */

#ifndef AVWRITER_H_
#define AVWRITER_H_

extern "C" {
	#include <libavcodec/avcodec.h>
	#include <libavformat/avformat.h>
	#include <libswscale/swscale.h>
}

#include <opencv2/opencv.hpp>

#include <boost/shared_ptr.hpp>
#include <boost/shared_array.hpp>

#include <pthread.h>

#include <sys/time.h>

#include <string>
#include <vector>
#include <stdint.h>

void* PROCESS_FRAMES_TO_WRITE(void* avwriter);

class AVWriter
{
	public:
		AVWriter(std::string filename,
				size_t src_width, size_t src_height,
				size_t dst_width, size_t dst_height,
				PixelFormat pix_fmt,
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
		PixelFormat			_pixFmt;
		
		int					_cpu;

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
		
		std::string			_avtiming;
		float				av_vid_time, av_scl_time, av_enc_time, av_wrt_time;
		
		std::deque<boost::shared_array<uint8_t> >	_frames_to_write;
		
		int write_frame(boost::shared_array<uint8_t> frame);
		friend void* PROCESS_FRAMES_TO_WRITE(void* avw);
};

#endif /*AVWRITER_H_*/

