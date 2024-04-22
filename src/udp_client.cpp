#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#define PORT 12345
#define ARRAY_SIZE 7
#define ADDRESS "127.0.0.1"
#define MAXLINE 1024

class UDPClient {

    private: 

    	int sockfd;
    	struct sockaddr_in servaddr;
        const char *hello = "Hello from the client!";
        char buffer_init[MAXLINE];
        char buffer[sizeof(double) * ARRAY_SIZE];
        
    public: 
    
    	void initClient() {	
    	
    	    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
                perror("socket creation failed");
                exit(EXIT_FAILURE);
    	    };

    	    memset(&servaddr, 0, sizeof(servaddr));
    
    	    servaddr.sin_family = AF_INET;
    	    servaddr.sin_port = htons(PORT);
    	    servaddr.sin_addr.s_addr = INADDR_ANY;
    
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

  
    	        std::cout << "Received Array: ";
    	        for (int i = 0; i < ARRAY_SIZE; ++i) {
        	    std::cout << receivedArray[i] << " ";
    	        };
    	        std::cout << std::endl;

      	   };
    
   	   close(sockfd);
    	   return 0;
    	};
};

int main() {
    UDPClient client;
    client.initClient();
    client.receiveFromServer();
    return 0;
}



