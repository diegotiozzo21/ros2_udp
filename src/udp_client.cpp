#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#define PORT 12345
#define ARRAY_SIZE 7
#define ADDRESS "192.168.131.2"
#define MAXLINE 1024


class PosePublisher {

	private:

		ros::NodeHandle nh;
		ros::Publisher pose_pub;
		geometry_msgs::PoseStamped pose_msg;
		double receivedArray[ARRAY_SIZE];
		
	public:

		PosePublisher() {
			pose_pub = nh.advertise<geometry_msgs::PoseStamped>("jetson_poses", 1);
		}
		
		void publishPose() {
			pose_msg.header.stamp = ros::Time::now();
			pose_msg.pose.position.x = receivedArray[0];
			pose_msg.pose.position.y = receivedArray[1];
			pose_msg.pose.position.z = receivedArray[2];
			pose_msg.pose.orientation.x = receivedArray[3];
			pose_msg.pose.orientation.y = receivedArray[4];
			pose_msg.pose.orientation.z = receivedArray[5];
			pose_msg.pose.orientation.w = receivedArray[6];
			pose_pub.publish(pose_msg);
			std::cout << pose_msg << std::endl;
		}
		
		void setReceivedArray(double *array) {
			memcpy(receivedArray, array, sizeof(receivedArray));
		}
};


class UDPClient {

    private: 

    	int sockfd;
    	struct sockaddr_in servaddr;
        const char *hello = "Hello from the client!";
        char buffer_init[MAXLINE];
        char buffer[sizeof(double) * ARRAY_SIZE];
		PosePublisher posePublisher;
        
    public: 
    
    	void initClient() {	
    	
    	    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
                perror("socket creation failed");
                exit(EXIT_FAILURE);
    	    };

    	    memset(&servaddr, 0, sizeof(servaddr));
    
    	    servaddr.sin_family = AF_INET;
    	    servaddr.sin_port = htons(PORT);
    	    servaddr.sin_addr.s_addr = inet_addr(ADDRESS); //INADDR_ANY;
    
    	    sendto(sockfd, (const char *)hello, strlen(hello), 
    		MSG_CONFIRM, (const struct sockaddr *) &servaddr, 
    		sizeof(servaddr));

	    socklen_t len;
    	    int n;
    	    n = recvfrom(sockfd, (char *)buffer_init, MAXLINE, 
    		MSG_WAITALL, (struct sockaddr *) &servaddr, &len);
    	    buffer_init[n] = '\0';
    	    printf("Server : %s\n", buffer_init);
    
    	    sendto(sockfd, (const char *)hello, strlen(hello), 
    		MSG_CONFIRM, (const struct sockaddr *) &servaddr, 
    		sizeof(servaddr));
    	};
    
    
    	int receiveFromServer() {
    	
    	    socklen_t len;
    	    ssize_t numBytesReceived;
    
            while(true) {
        	    	
    		numBytesReceived = recvfrom(sockfd, buffer, sizeof(buffer), 0, (struct sockaddr *)&servaddr, &len);
    	
    		if (numBytesReceived < 0) {
        	    perror("recvfrom failed");
        	    close(sockfd);
        	    return -1;
    	        };
    
	
    	        if (numBytesReceived != sizeof(double) * ARRAY_SIZE) {
        	   std::cerr << "Incomplete data received\n";
        	   continue;
    	        };

    	        double receivedArray[ARRAY_SIZE];
    	        memcpy(receivedArray, buffer, sizeof(receivedArray));

				posePublisher.setReceivedArray(receivedArray);	 
				posePublisher.publishPose();
      	   };
    
   	   close(sockfd);
    	   return 0;
    	};
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "jetson_orin_poses_node");
    UDPClient client;
    client.initClient();
    client.receiveFromServer();
    return 0;
}



