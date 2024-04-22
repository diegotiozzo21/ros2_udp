// Server side implementation of UDP client-server model 
#include <bits/stdc++.h> 
#include <stdlib.h> 
#include <unistd.h> 
#include <string.h> 
#include <sys/types.h> 
#include <sys/socket.h> 
#include <arpa/inet.h> 
#include <netinet/in.h> 
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <memory>
using std::placeholders::_1;

#define PORT 12345 
#define ARRAY_SIZE 7 
#define ADDRESS "127.0.0.1"
#define MAXLINE 1024

class UDPServer {

    private:

        int sockfd;
	struct sockaddr_in servaddr, cliaddr;
	const char *hello = "Hello from the server!";
	char buffer_init[MAXLINE];

    public:
	
	void initServer() {
           
	    sockfd = socket(AF_INET, SOCK_DGRAM, 0);	
            if ( sockfd  < 0 ) { 
                perror("socket creation failed"); 
                exit(EXIT_FAILURE); 
            };

            memset(&servaddr, 0, sizeof(servaddr)); 
            memset(&cliaddr, 0, sizeof(cliaddr)); 

            servaddr.sin_family = AF_INET; // IPv4 
            servaddr.sin_addr.s_addr = INADDR_ANY; 
            servaddr.sin_port = htons(PORT); 

           if ( bind(sockfd, (const struct sockaddr *)&servaddr,  
                sizeof(servaddr)) < 0 ) { 
               perror("bind failed"); 
               exit(EXIT_FAILURE); 
           }; 
           
           socklen_t len_init;
           int n;
           len_init = sizeof(cliaddr);
           n = recvfrom(sockfd, (char *)buffer_init, MAXLINE, 
           			MSG_WAITALL, ( struct sockaddr *) &cliaddr, &len_init);
           buffer_init[n] = '\0';
           printf("Client : %s\n", buffer_init);
           sendto(sockfd, (const char *)hello, strlen(hello), 
           	MSG_CONFIRM, (const struct sockaddr *) &cliaddr, len_init);
           
	};

	void sendToClient(const char* buffer) const {           
	    ssize_t bufferLength = strlen(buffer);
	    int success = (sendto(sockfd, buffer, bufferLength, 0, (const struct sockaddr *)&cliaddr, sizeof(cliaddr)));
	    if (success < 0){
	        perror("send to failed");
	    };
	};

};

class UDPSubscriber : public rclcpp::Node {

    public:

        UDPServer server;
        
        void send_message (double message[ARRAY_SIZE]) const {
            char buffer[sizeof(double)*ARRAY_SIZE];
	    memcpy(buffer, message, sizeof(double)*ARRAY_SIZE);
	    server.sendToClient(buffer);
        }
  
        UDPSubscriber() : Node("udp_pose_subscriber")  {
	  server.initServer();
          subscription_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
          "poses", 10, std::bind(&UDPSubscriber::topic_callback, this, _1));
        }
    
 
    private:
    
        void topic_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg) const {
           for(auto pose : msg-> poses){
              auto p = pose.position;
      	      auto q = pose.orientation;
	      double message[ARRAY_SIZE] = {p.x, p.y, p.z, q.x, q.y, q.z, q.w};
	      //RCLCPP_INFO(this->get_logger(), "Sending...\n Position: (%f, %f, %f), Orientation: (%f, %f, %f, %f)", 
	      //	p.x, p.y, p.z, 
	      //		q.x, q.y, q.z, q.w);
              send_message(message);
             }  
         }
        rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription_;
};


int main(int argc, char *argv[]) {
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<UDPSubscriber>());
        rclcpp::shutdown();
	return 0;
}
