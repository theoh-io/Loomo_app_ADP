#ifndef SOCKET_SERVER_H_
#define SOCKET_SERVER_H_

#include <thread>
#include <mutex>
#include <string>
#include <opencv2/opencv.hpp>

namespace ninebot_algo
{
	/*! namespace of this algorithm  */
	namespace testing_algo
	{

		class SocketServer {
		public :
			SocketServer(int port);
			SocketServer(bool is_send_timeout, bool is_recv_timeout, int port);
			virtual ~SocketServer();

			int recvChars(char* recv_chars, const int length);
			int sendChars(const char* send_chars, const int length);
			std::string receiveLine();
			int sendLine(std::string);
			bool isConnected();
			bool isStopped();
			bool stopSocket();
			int  getReconnectTimes();

			// call example
			void sendMessage();
			void receiveMessage();
			void sendChars();
			void receiveChars();

			int recvFloats(float* recv_floats, const int length);
			int recvFloatsP(float* recv_floatsP, const int lengthP);
			int sendFloats(const float* send_floats, const int length);
			int sendImage(cv::Mat image, const int width, const int height, const int channels=3, const int bags=1);

		private :
			int run();
			int init_socket(int port);
			void re_connected();
			void reset_connect();

		private:
			const static int RECEIVE_BUFFER_MAX = 200000;
			std::thread m_run_thread;
			std::mutex _mutex_socket_connected;
			std::mutex _mutex_socket_stopped;

			int _socket_handler;
			int _new_socket;
			int _process_socket;
			int _init_success;
			int _reconnect_times;
			bool _socket_connected;
			bool _socket_stopped;
			bool m_is_send_timeout;
			bool m_is_recv_timeout;
		};
		
	}
}

#endif  
