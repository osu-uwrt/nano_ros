# protocol

- let the clients assign topic ids in range 0-255 where 128-255 is for publishers, 0-127 is for subscribers 

## data serialization
- nano_ros_codegen is responsible for looking at topics and generating code to handle serde of data.
- it will also create structures for the types that the user can use, e.g. a subscription handler returning a sensor_msgs/msg/Imu or an embedded client population a sensor_msgs/msg/Imu to be published
- the end product of the codegen is as follows
    - nano_ros_publish(int topic_id, <ros_type *> data)
    - nano_ros_init_subscriber(int topic_id, sub_handler_t callback)
        - where sub_handler_t is a pointer to function returning error if any and parameters (void *in_data)

## wire packet types
- REGISTER
    - packet for registering a topic 
    - contains type of topic (pub/sub), type name string e.g. std_msgs__msg__string, qos, and name of the topic as a string
- DATA
    - data going to host from embedded client
- ACK 
    - status code 
## client -> host:

## host -> client:
