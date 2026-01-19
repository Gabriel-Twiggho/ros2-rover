# --- ROS and audio imports
import rclpy # ROS 2 python client library
from rclpy.node import Node # Base class for creating a ROS 2 node
from std_msgs.msg import Int16MultiArray #Standard message type for publishing arrays of 16 bit integers
# --- External libraries for audio input and array handling
import sounddevice as sd
import numpy as np #For handling audio data as arrays, we use .flatten() function from numpy

# --- Define the ROS node class
class AudioCaptureNode(Node):
	"""
	A ROS 2 node that captures audio froma microphone and publishes it to a topic
	"""
	def __init__(self):

		super().__init__('audio_capture_node') #calls init in rclpy.node.Node class which sets up Node called 'audio_capture_node'
		
		
		#defines ros parameters , handy as they can be changed on console at runtime
		self.declare_parameter('sample_rate', 16000) 
		self.declare_parameter('channels',1) #
		self.declare_parameter('chunk_size', 1600) 
		
		#defines python variables
		self.sample_rate = self.get_parameter('sample_rate').value
		self.channels = self.get_parameter('channels').value
		self.chunk_size = self.get_parameter('chunk_size').value
		
		#publishing to topic "audio_raw"
		self.publisher_object = self.create_publisher(
			Int16MultiArray, 
			'audio_raw', 	 
			10				
		)
		
		#calculate the seconds in which a chunk will be done based on the sample rate
		timer_period = self.chunk_size / self.sample_rate 
		#calls the capture and publish function every time period
		self.timer = self.create_timer(timer_period, self.capture_and_publish)
		

		try:
			self.stream = sd.InputStream(
				samplerate=self.sample_rate, 
				channels=self.channels,		 
				dtype='int16',				 
				blocksize=self.chunk_size	 
			)
			self.stream.start() 
			self.get_logger().info('Microphone stream started successfully')
			
		except Exception as e:
			self.get_logger().error(f'Failed to open audio stream: {e}')
			rclpy.shutdown()
			

	def capture_and_publish(self):
		"""
		Called periodically by the ROS timer.
		Captures audio chunk from mic and publishes it to 'audio_raw'.
		"""
		try:
			#uses soundevice library to read audio data and boolean overflowed aka if we are reading to slowly from sounddevice
			audio_data, overflowed = self.stream.read(self.chunk_size)
			
			
			if overflowed:
				self.get_logger().warn('Audio input overflowed!')
				
			#we need to flatten to 1D and covnert from array to list -> Int16MultiArray variable
			msg = Int16MultiArray()
			msg.data = audio_data.flatten().tolist()
			
			#publish
			self.publisher_object.publish(msg)
			
			#optional log publishing (commented out to redue spam)
			#self.get_logger().info(f'Published audio chunk of size {Len(msg.data)}')
			
		except Exception as e:
			self.get_logger().error(f'Error during audio capture: {e}')
			

def main(args=None):

	#sets up ros2 infrastructure must be called before any nodes
	rclpy.init(args=args)
	#creates object
	audio_capture_node = AudioCaptureNode()
	
	
	try: 
		#brings all the ros commands to life
		rclpy.spin(audio_capture_node)
	except KeyboardInterrupt:
		
		pass
	finally:
		
		if 'stream' in audio_capture_node.__dict__ and audio_capture_node.stream:
			audio_capture_node.stream.stop()
			audio_capture_node.stream.close()
				
		audio_capture_node.destroy_node()
		rclpy.shutdown()
			
if __name__ == '__main__':
	main()
			
