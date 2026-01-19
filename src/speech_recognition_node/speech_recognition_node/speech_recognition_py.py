# --- ROS and Audio recognition imports
import rclpy #ROS 2 client library
from rclpy.node import Node # base class for ROS 2 nodes
from std_msgs.msg import Int16MultiArray, String #messages types for audio data and recognised text

# --- system and vosk imports
import os #for file path handling
import json #to parse vosks JSON output
import vosk #vosk speech recognition engine
import numpy as np # to convert audio lists to byte arrays

# -
class SpeechRecognitionNode(Node):
    """
    A ros 2 node that subscribes to raw audio from /audio_raw,
    performs speech to text conversion using vosk engine,
    and publishes recognised phrases to /speech_text
    """
    
    def __init__(self):
        super().__init__('speech_recognition_node')
        

        model_path = os.path.expanduser("~/models/vosk-model-small-en-us-0.15") # path to vosk model
        

        if not os.path.exists(model_path):
            self.get_logger().error(f"vosk model not found at {model_path}. Shutting down")
            rclpy.shutdown()
            return
            

        try:
            self.model = vosk.Model(model_path) 
            #creates vosk speech recogniser object
            self.recognizer = vosk.KaldiRecognizer(self.model, 16000)
            self.get_logger().info('Vosk model loaded successfully.')
        except Exception as e:
            self.get_logger().error(f'Failed to load Vosk model: {e}')
            rclpy.shutdown()
            return
            
        #key difference between publisher variable is that subscription variable it calls a function when data comes in
        self.subscription = self.create_subscription(
            Int16MultiArray,    #message type
            'audio_raw',        #topic name
            self.audio_callback,#callback function when data arrives
            10                  # queue size (QOS depth)
        )
        
        #initialise new publisher "speech_text"
        self.publisher_ = self.create_publisher(String, 'speech_text', 10)

        self.get_logger().info('Speech recognition node has started.')
    

    def audio_callback(self, msg):
        """
        callback triggered when new audio data arrives on /audio_raw.
        converts the raw audio list into bytes and passes it to the recogniser.
        if a phrase is recognised, it is published as a text message.
        """

        #convert message data into something that vosk can understand
        audio_bytes = np.array(msg.data, dtype=np.int16).tobytes()
        
        #true if vosk thinks it has enough for a full phrase
        if self.recognizer.AcceptWaveform(audio_bytes):
            #gets final recognizable result as a JSON string
            result_json = self.recognizer.Result() 
            #converts JSON string into normal python dict with they key of dict being 'text'
            result_dict = json.loads(result_json) 
            
            #if there is actually value in the text key 
            if 'text' in result_dict and result_dict['text']:
                #get the recognised text from 'text' key in dict
                recognized_text = result_dict['text']
                self.get_logger().info(f'Recognized: "{recognized_text}"') 
                
                #create ROS 2 variable 
                text_msg = String()
                #set data attribute to recognised_text
                text_msg.data = recognized_text
                #publish it
                self.publisher_.publish(text_msg)
            

def main(args=None):
    #sets up ROS 2 infrastructure
    rclpy.init(args=args) 
    
    if rclpy.ok():
        #creates object
        speech_recognition_node = SpeechRecognitionNode()

        try:
            rclpy.spin(speech_recognition_node) 
        except KeyboardInterrupt:
            pass 
        finally:
            speech_recognition_node.destroy_node() 
            rclpy.shutdown() 

if __name__ == '__main__':
    main()
