// *****************************************************************************
//
// © Copyright 2020, Septentrio NV/SA.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//    1. Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//    2. Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//    3. Neither the name of the copyright holder nor the names of its
//       contributors may be used to endorse or promote products derived
//       from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE. 
//
// *****************************************************************************

// *****************************************************************************
//
// Boost Software License - Version 1.0 - August 17th, 2003
// 
// Permission is hereby granted, free of charge, to any person or organization
// obtaining a copy of the software and accompanying documentation covered by
// this license (the "Software") to use, reproduce, display, distribute,
// execute, and transmit the Software, and to prepare derivative works of the
// Software, and to permit third-parties to whom the Software is furnished to
// do so, all subject to the following:

// The copyright notices in the Software and this entire statement, including
// the above license grant, this restriction and the following disclaimer,
// must be included in all copies of the Software, in whole or in part, and
// all derivative works of the Software, unless such copies or derivative
// works are solely in the form of machine-executable object code generated by
// a source language processor.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT
// SHALL THE COPYRIGHT HOLDERS OR ANYONE DISTRIBUTING THE SOFTWARE BE LIABLE
// FOR ANY DAMAGES OR OTHER LIABILITY, WHETHER IN CONTRACT, TORT OR OTHERWISE,
// ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.
//
// *****************************************************************************

#include <thread>
#include <functional>
// Boost includes
#include <boost/algorithm/string/join.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/system/error_code.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

// ROS includes
#include "rclcpp/rclcpp.hpp"
// ROSaic includes
#include <septentrio_gnss_driver/communication/circular_buffer.hpp>

#ifndef ASYNC_MANAGER_HPP
#define ASYNC_MANAGER_HPP

/**
 * @file async_manager.hpp
 * @date 20/08/20
 * @brief Implements asynchronous operations for an I/O manager
 *
 * Such operations include reading NMEA messages and SBF blocks yet also sending commands to serial port or via TCP/IP.
 */
 
namespace io_comm_rx
{
	
	/**
	 * @class Manager
	 * @brief Interface (in C++ terms), that could be used for any I/O manager, synchronous and asynchronous alike
	 */
	class Manager {
		public:
			typedef std::function<void(const uint8_t*, std::size_t&)> Callback;
			virtual ~Manager() {}
			//! Sets the callback function
			virtual void setCallback(const Callback& callback) = 0;
			//! Sends commands to the receiver
			virtual bool send(std::string cmd, std::size_t size) = 0;
			//! Waits count seconds before throwing ROS_INFO message in case no message from the receiver arrived
			virtual void wait(uint16_t* count) = 0;
			//! Determines whether or not the connection is open
			virtual bool isOpen() const = 0;
	};


	/**
	 * @class AsyncManager
	 * @brief This is the central interface between ROSaic and the Rx(s), managing I/O operations such as reading messages and sending commands..
	 * 
	 * StreamT is either boost::asio::serial_port or boost::asio::tcp::ip
	 */
	template <typename StreamT>
	class AsyncManager : public Manager 
	{
		public:
			/**
			 * @brief Class constructor
			 * @param stream Whether TCP/IP or serial communication, either boost::asio::serial_port or boost::asio::tcp::ip
			 * @param io_service The io_context object. The io_context represents your program's link to the operating system's I/O services 
			 * @param[in] buffer_size Size of the circular buffer in bytes
			 */
			AsyncManager(std::shared_ptr<StreamT> stream, std::shared_ptr<boost::asio::io_service> io_service, std::size_t buffer_size = 8192);
			virtual ~AsyncManager();
			
			/**
			 * @brief Allows to connect to the CallbackHandlers class
			 * @param callback The function that becomes our callback, typically the readCallback() method of CallbackHandlers
			 */
			void setCallback(const Callback& callback) { read_callback_ = callback; }
			
			void wait(uint16_t* count);
			
			/**
			 * @brief Sends commands via the I/O stream.
			 * @param cmd The command to be sent
			 * @param size The size of the command
			 */
			bool send(std::string cmd, std::size_t size);
			
			bool isOpen() const { return stream_->is_open(); }
			
		protected:
			
			//! Reads in via async_read_some and hands certain number of bytes (bytes_transferred) over to async_read_some_handler 
			void read();
			
			//!  Handler for async_read_some (Boost library)..
			void asyncReadSomeHandler(const boost::system::error_code& error, std::size_t bytes_transferred);
			
			//! Sends command "cmd" to the Rx
			void write(std::string cmd, std::size_t size);
			
			//! Closes stream "stream_"
			void close();
			
			//! Tries parsing SBF/NMEA whenever the boolean class variable "try_parsing" is true
			void tryParsing();
			
			//! Mutex to control changes of class variable "try_parsing"
			std::mutex parse_mutex_;
			
			//! Determines when the tryParsing() method will attempt parsing SBF/NMEA
			bool try_parsing_;
			
			//! Determines when the asyncReadSomeHandler() method should write SBF/NMEA into the circular buffer
			bool allow_writing_;
			
			//! Condition variable complementing "parse_mutex"
			std::condition_variable parsing_condition_;
			
			//! Stream, represents either serial or TCP/IP connection
			std::shared_ptr<StreamT> stream_;
			
			//! io_context object
			std::shared_ptr<boost::asio::io_service> io_service_;
			
			//! Buffer for async_read_some() to read continuous SBF/NMEA stream
			std::vector<uint8_t> in_; 
			
			//! Circular buffer to avoid unsuccessful SBF/NMEA parsing due to incomplete messages 
			CircularBuffer circular_buffer_; 
			
			//! Memory location where read_callback_ will start reading unless part of SBF/NMEA had to be appended before
			uint8_t * to_be_parsed_;
			
			//! New thread for receiving incoming messages
			std::shared_ptr<std::thread> async_background_thread_;
			std::shared_ptr<std::thread> try_parsing_thread_ptr_;
			
			//! Callback to be called once message arrives
			Callback read_callback_; 
			
			//! Whether or not we want to sever the connection to the Rx
			bool stopping_; 
			
			/// Size of in_ buffers
			const std::size_t buffer_size_;
			
			//! Boost timer for throwing ROS_INFO message once timed out due to lack of incoming messages
			boost::asio::deadline_timer timer_;
			
			//! Number of seconds before ROS_INFO message is thrown (if no incoming message)
			const uint16_t count_max_;
			
			//! Handles the ROS_INFO throwing (if no incoming message)
			void callAsyncWait(uint16_t* count);
			
			//! Number of times the DoRead() method has been called (only counts initially)
			uint16_t do_read_count_;
	};
	 
	 
	template <typename StreamT>
	void AsyncManager<StreamT>::tryParsing()
	{
		uint8_t * to_be_parsed;
		to_be_parsed = new uint8_t[buffer_size_];
		to_be_parsed_ = to_be_parsed;
		bool timed_out = false;
		std::size_t shift_bytes = 0;
		std::size_t arg_for_read_callback = 0;
		
		while(!timed_out) // Loop will stop if condition variable timed out
		{
			std::unique_lock<std::mutex> lock(parse_mutex_);
			parsing_condition_.wait_for(lock, std::chrono::seconds(10), [this](){return try_parsing_;});
			bool timed_out = !try_parsing_;
			if (timed_out) break;
			try_parsing_ = false;
			allow_writing_ = true;
			std::size_t current_buffer_size = circular_buffer_.size();
			arg_for_read_callback += current_buffer_size;
			circular_buffer_.read(to_be_parsed + shift_bytes, current_buffer_size);
			
			lock.unlock();
			parsing_condition_.notify_one();
			
			try
			{
				RCLCPP_DEBUG(rclcpp::get_logger("async_manager"), "Calling read_callback_() method, with number of bytes to be parsed being %li",
					arg_for_read_callback);
				read_callback_(to_be_parsed_, arg_for_read_callback);
			}
			catch (std::size_t& parsing_failed_here) 
			{
				to_be_parsed_ = to_be_parsed + parsing_failed_here;
				RCLCPP_DEBUG(rclcpp::get_logger("async_manager"), "Current buffer size is %li and parsing_failed_here is %li", current_buffer_size,
					parsing_failed_here);
				arg_for_read_callback = arg_for_read_callback - parsing_failed_here;
				if (arg_for_read_callback < 0) // In case some parsing error was not caught, which should never happen..
				{
					delete [] to_be_parsed; // Freeing memory
					to_be_parsed = new uint8_t[buffer_size_];
					to_be_parsed_ = to_be_parsed;
					shift_bytes = 0;
					arg_for_read_callback = 0;
					continue;
				}
				shift_bytes += current_buffer_size;
				continue;
			}
			delete [] to_be_parsed; // Freeing memory
			to_be_parsed = new uint8_t[buffer_size_];
			to_be_parsed_ = to_be_parsed;
			shift_bytes = 0;
			arg_for_read_callback = 0;
		}
		RCLCPP_INFO(rclcpp::get_logger("async_manager"), "TryParsing() method finished since it did not receive anything to parse for 10 seconds..");
	}
	
	
	template <typename StreamT>
	bool AsyncManager<StreamT>::send(std::string cmd, std::size_t size) 
	{
		if(size == 0) 
		{
			RCLCPP_ERROR(rclcpp::get_logger("async_manager"), "Message size to be sent to the Rx would be 0");
			return true;
		}
		
		std::vector<uint8_t> vector_temp(cmd.begin(), cmd.end());
		uint8_t *p = &vector_temp[0];
		
		io_service_->post(boost::bind(&AsyncManager<StreamT>::write, this, cmd, size));
		return true;
	}
	
	template <typename StreamT>
	void AsyncManager<StreamT>::write(std::string cmd, std::size_t size) 
	{
		boost::asio::write(*stream_, boost::asio::buffer(cmd.data(), size));
		// Prints the data that was sent
		RCLCPP_DEBUG(rclcpp::get_logger("async_manager"), "Sent the following %li bytes to the Rx: \n%s", size, cmd.c_str());
	}

	template <typename StreamT>
	void AsyncManager<StreamT>::callAsyncWait(uint16_t* count)
	{
		timer_.async_wait(boost::bind(&AsyncManager::wait, this, count));
	}
	
	template <typename StreamT>
	AsyncManager<StreamT>::AsyncManager(std::shared_ptr<StreamT> stream,
			std::shared_ptr<boost::asio::io_service> io_service,
			std::size_t buffer_size) : timer_(*(io_service.get()), boost::posix_time::seconds(1)), 
			stopping_(false), try_parsing_(false), allow_writing_(true), do_read_count_(0), buffer_size_(buffer_size), 
			count_max_(6), circular_buffer_(buffer_size) 
			// Since buffer_size = 8912 in declaration, no need in definition any more (even yields error message, 
			// since "overwrite").
	{
		RCLCPP_DEBUG(rclcpp::get_logger("async_manager"), "Setting the private stream variable of the AsyncManager instance.");
		stream_ = stream;
		io_service_ = io_service;
		in_.resize(buffer_size_);
		 
		io_service_->post(boost::bind(&AsyncManager<StreamT>::read, this));
		// This function is used to ask the io_service to execute the given handler, but without allowing the io_service
		// to call the handler from inside this function. The function signature of the handler must be: void handler(); 
		// The io_service guarantees that the handler (given as parameter) will only be called in a thread in which the 
		// run(), run_one(), poll() or poll_one() member functions is currently being invoked. So the fundamental 
		// difference is that dispatch will execute the work right away if it can and queue it otherwise while post queues the work no matter what.
		async_background_thread_.reset(new std::thread(boost::bind(&boost::asio::io_service::run, io_service_)));
		// Note that io_service_ is already pointer, hence need dereferencing operator & (ampersand). If the value of the 
		// pointer for the current thread is changed using reset(), then the previous value is destroyed by calling the 
		// cleanup routine. Alternatively, the stored value can be reset to NULL and the prior value returned by calling
		// the release() member function, allowing the application to take back responsibility for destroying the object. 
		uint16_t count = 0;
		//boost::thread(boost::bind(&AsyncManager::callAsyncWait, this, &count));
		
		RCLCPP_DEBUG(rclcpp::get_logger("async_manager"), "Launching tryParsing() thread..");
		try_parsing_thread_ptr_.reset(new std::thread(std::bind(&AsyncManager::tryParsing, this)));
	} 	// Calls std::terminate() on thread just created
	 
	template <typename StreamT>
	AsyncManager<StreamT>::~AsyncManager() 
	{
		async_background_thread_->join(); 
	}

	template <typename StreamT>
	void AsyncManager<StreamT>::read() 
	{
		stream_->async_read_some(
								boost::asio::buffer(in_.data(),
								in_.size()),
								boost::bind(&AsyncManager<StreamT>::asyncReadSomeHandler, this,
								boost::asio::placeholders::error,
								boost::asio::placeholders::bytes_transferred));
								// The handler is async_read_some_handler, whose call is postponed to 
								// when async_read_some completes.
		if (do_read_count_ < 5) ++do_read_count_;
	}
	 
	template <typename StreamT>
	void AsyncManager<StreamT>::asyncReadSomeHandler(const boost::system::error_code& error,
								std::size_t bytes_transferred) 
	{
		if (error) 
		{
			RCLCPP_ERROR(rclcpp::get_logger("async_manager"), "Rx ASIO input buffer read error: %s, %li", error.message().c_str(), bytes_transferred);
		} 
		else if (bytes_transferred > 0) 
		{ 
			if (read_callback_) //Will be false in InitializeSerial (first call) since read_callback_ not added yet..
			{
				std::unique_lock<std::mutex> lock(parse_mutex_);
				parsing_condition_.wait(lock, [this](){return allow_writing_;});
				circular_buffer_.write(in_.data(), bytes_transferred);
				allow_writing_ = false;
				try_parsing_ = true;
				lock.unlock();
				parsing_condition_.notify_one();
				std::vector<uint8_t> empty;
				in_ = empty;
				in_.resize(buffer_size_);
			}
		}
	 
		if (!stopping_)
			io_service_->post(boost::bind(&AsyncManager<StreamT>::read, this));
	}
	 
	template <typename StreamT>
	void AsyncManager<StreamT>::close() 
	{
		stopping_ = true;
		boost::system::error_code error;
		stream_->close(error); 
		if(error)
		{
			RCLCPP_ERROR_STREAM(rclcpp::get_logger("async_manager"), "Error while closing the AsyncManager: " << error.message().c_str());
		}
	}
	 
	template <typename StreamT>
	void AsyncManager<StreamT>::wait(uint16_t* count) 
	{
		if (*count < count_max_)
		{
			++(*count);
			timer_.expires_at(timer_.expires_at() + boost::posix_time::seconds(1));
			if (!(*count == count_max_))
			{
				timer_.async_wait(boost::bind(&AsyncManager::wait, this, count));
			}
		}
		if ((*count == count_max_) && (do_read_count_ < 3))
		// Why 3? Even if there are no incoming messages, read() is called once. 
		// It will be called a second time in TCP/IP mode since (just example) "IP10<" is transmitted.
		{
			RCLCPP_INFO(rclcpp::get_logger("async_manager"), "No incoming messages, driver stopped, ros::spin() will spin forever unless you hit Ctrl+C.");
			//async_background_thread_->interrupt();
		}
	}
}
 
#endif // for ASYNC_MANAGER_HPP
 
 
 