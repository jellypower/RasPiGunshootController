#define DEV_INPUT_EVENT "/dev/input"
#define EVENT_DEV_NAME "event"

#include "RTIMULib.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <time.h>
#include <stdlib.h>
#include <linux/input.h>
#include <linux/fb.h>
#include <string.h>
#include <dirent.h>
#include <poll.h>

#define ACK "ackfromunity"

struct packetData{
	RTFLOAT angle[4];
	RTFLOAT devSpeed;
	int btnClick;
};

struct host_info{
	int sd;
	struct sockaddr_in recver;
};


//----------------------------------- global
int PORTNO;
RTQuaternion qStart;
int btnDir;

//----------------------------------- global

struct host_info initSocket();
int sndPkt(struct host_info *host, struct packetData* data);
void updateVRData(struct packetData *data, RTIMU_DATA *sensor, struct pollfd *joyfd, int sampleRate);
struct pollfd initJoystick();
int getJoystickDir(struct pollfd* evpoll);

int main(int argc, char* argv[])
{

    if(argc >= 2){
        PORTNO = atoi(argv[1]);
    }
    else{
        PORTNO = 9002;
    }
    

    printf("%d\n",PORTNO);

    int sampleCount = 0;
    int sampleRate = 0;
    uint64_t rateTimer;
    uint64_t displayTimer;
    uint64_t now;

    //---------------------------------------------- data struct init

    struct packetData data;
    struct host_info host = initSocket();

    struct pollfd joyfd = initJoystick();
    
    memset((char*)&data, '\0', sizeof(data));
    
    //---------------------------------------------- 





    RTIMUSettings *settings = new RTIMUSettings("RTIMULib");

    RTIMU *imu = RTIMU::createIMU(settings);

    if ((imu == NULL) || (imu->IMUType() == RTIMU_TYPE_NULL)) {
        printf("No IMU found\n");
        exit(1);
    }

    imu->IMUInit();

    imu->setSlerpPower(0.02);
    imu->setGyroEnable(true);
    imu->setAccelEnable(true);
    imu->setCompassEnable(true);

    rateTimer = displayTimer = RTMath::currentUSecsSinceEpoch();


    //---------------------------------------- imu setting init
    imu->IMURead();
    
    qStart = imu->getIMUData().fusionQPose;


    qStart.setX(0);
    qStart.setY(0);
    qStart.normalize();
    qStart = qStart.conjugate();

    //-------------------------------------------------------


    while (1) {

        usleep(imu->IMUGetPollInterval() * 1000);

        while (imu->IMURead())
	      {
            RTIMU_DATA imuData = imu->getIMUData();
            sampleCount++;

            now = RTMath::currentUSecsSinceEpoch();


            if ((now - displayTimer) > 100000) {
                //printf("1: Sample rate %d: %s\n", sampleRate, RTMath::displayDegrees("", imuData.fusionPose));
                
                //--------------------------------------------------------------- custom
                
                	
                updateVRData(&data , &imuData, &joyfd, sampleRate);
            		sndPkt(&host, &data);


                //--------------------------------------------------------------- custom

                displayTimer = now;
		}
		
            

            if ((now - rateTimer) > 1000000) {
                sampleRate = sampleCount;
                sampleCount = 0;
                rateTimer = now;
            }

	}
    }
}



struct host_info initSocket(){
	struct timeval pollInterval = {1, 0};
	int pollTrials = 10;
	int i;

	int sd;
	int fBroadcast=1;
	char buf[256];
	struct sockaddr_in sin;
	unsigned int sinRecvLen;

	if((sd = socket(AF_INET, SOCK_DGRAM, 0)) == -1){
		perror("socket");
		exit(1);
	}

	if(setsockopt(sd, SOL_SOCKET, SO_BROADCAST, (const char*)&fBroadcast, sizeof(fBroadcast))==-1){
		perror("setsockopt");
		exit(1);
	}


	if(setsockopt(sd, SOL_SOCKET, SO_RCVTIMEO, &pollInterval, sizeof(pollInterval))== -1){
		perror("setsockopt");
		exit(1);
	}


	memset((char*)&sin, '\0', sizeof(sin));
	sin.sin_family = AF_INET;
	sin.sin_port = htons(PORTNO);
	sin.sin_addr.s_addr = inet_addr("255.255.255.255");

	strcpy(buf,"synctounity");


	sinRecvLen = sizeof(sin);

	for(i=0;i<pollTrials;i++){

		printf("send sync msg...\n");
		if((sendto(sd, buf, strlen(buf)+1, 0,
		(struct sockaddr*)&sin, sizeof(sin))) == -1)
		{
			perror("sendto");
			exit(1);
		}
	
		if((recvfrom(sd, buf, 255,0,
		(struct sockaddr*)&sin, &sinRecvLen)) == -1){
			perror("recvfrom");
			if(i<pollTrials-1)
				continue;
			else{
				perror("recvfrom[timeout]");
				exit(1);
			}
		}
		
		break;

	}

	printf("[%d]:connected!!\n", strcmp(buf, ACK));
	if((sendto(sd, buf, strlen(buf)+1, 0,
	(struct sockaddr*)&sin, sizeof(sin))) == -1){
		perror("sendto");
		exit(1);
	}


	struct host_info host_info;

	host_info.sd = sd;
	host_info.recver = sin;

	return host_info;
}

int sndPkt(struct host_info *host, struct packetData* data){

		
	if((sendto(host->sd, (char*)data, sizeof(*data), 0,
	(struct sockaddr*)&(host->recver), sizeof(host->recver))) == -1){
		perror("sendto");
		return -1;
	}

	return 0;

}

void updateVRData(struct packetData *data, RTIMU_DATA *sensor, struct pollfd *joyfd, int sampleRate){

  //-------------------------- settings
	sampleRate = sampleRate == 0 ? 44:sampleRate;
		
	float eTime = 1.0/sampleRate;
 
  RTQuaternion deg = sensor->fusionQPose;
  
  //------------------------------ calc linear Accel and update devSpeed
  
  RTVector3 gravityAccel;
  RTVector3 linearAccel;
  
  gravityAccel.setX(-sin(deg.y()));
  gravityAccel.setY(sin(deg.x())*cos(deg.y()));
  gravityAccel.setZ(cos(deg.x())*cos(deg.y()));

  linearAccel = sensor->accel;
  linearAccel-=gravityAccel;
  
  data->devSpeed = linearAccel.length();
  
  //----------------------------- update devangle
  printf("1: %f %f %f %f\n", deg.x(), deg.y(), deg.z(), deg.scalar());
  printf("2: %f %f %f %f\n", qStart.x(), qStart.y(), qStart.z(), qStart.scalar());
  deg= qStart * deg;
  deg.normalize();
  printf("3: %f %f %f %f\n", deg.x(), deg.y(), deg.z(), deg.scalar());
  
  data->angle[0] = deg.x() ;
  data->angle[1] = deg.y() ;
  data->angle[2] = deg.z() ;
  data->angle[3] = deg.scalar();
		
  //------------------------------ update 

  data->btnClick = getJoystickDir(joyfd);
	
}



//----------------------------------------------------------------------------------------------------------------- joystick

static int is_event_device(const struct dirent *dir)
{
	return strncmp(EVENT_DEV_NAME, dir->d_name,
		       strlen(EVENT_DEV_NAME)-1) == 0;
}

static int open_evdev(const char *dev_name)
{
	struct dirent **namelist;
	int i, ndev;
	int fd = -1;

	ndev = scandir(DEV_INPUT_EVENT, &namelist, is_event_device, versionsort);
	if (ndev <= 0)
		return ndev;

	for (i = 0; i < ndev; i++)
	{
		char fname[64];
		char name[256];

		snprintf(fname, sizeof(fname),
			 "%s/%s", DEV_INPUT_EVENT, namelist[i]->d_name);
		fd = open(fname, O_RDONLY); // Open with read only
		if (fd < 0)
			continue;
		ioctl(fd, EVIOCGNAME(sizeof(name)), name);


		//printf("%s\n", namelist[i]->d_name);

		if (strcmp(dev_name, name) == 0)
			break;
		close(fd);
	}

	for (i = 0; i < ndev; i++)
		free(namelist[i]);

	return fd;
}

struct pollfd initJoystick(){
  
    struct pollfd evpoll = {
		  .events = POLLIN,
	  };
  
    evpoll.fd = open_evdev("Raspberry Pi Sense HAT Joystick"); // save joystick event file descriptor  to evpoll.fd
	  if (evpoll.fd < 0) {
		    fprintf(stderr, "Event device not found.\n");
		    exit(1);
	}
 
  return evpoll;
}


int getJoystickDir(struct pollfd* evpoll){


  struct input_event ev[64]; // this system use input_event
  int i, rd;

  while (poll(evpoll, 1, 0) > 0){
  
	  rd = read(evpoll->fd, ev, sizeof(struct input_event) * 64);
	  if (rd < (int) sizeof(struct input_event)) { // read input info from joystick
	  	fprintf(stderr, "expected %d bytes, got %d\n",
	  	        (int) sizeof(struct input_event), rd);
	  	return -1;
	  }

     
	  for (i = 0; i < rd / sizeof(struct input_event); i++) {
	  	if (ev->type != EV_KEY)
	  		continue;
	  	if (ev->value == 0)
	  		continue;
	  	switch (ev->code) {  // read input from joystick
	  		case KEY_ENTER:
	  			return 0;
          break;
        case KEY_UP:
          return 1;
	  			break;
        case KEY_DOWN:
          return 2;
	  			break;
        case KEY_LEFT:
          return 3;
	  			break;
        case KEY_RIGHT:
          return 4;
          break;
        default:
          return -1;
		  		break;
		  }
	  }
  }
  return -1;
}





