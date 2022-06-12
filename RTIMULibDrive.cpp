#include "RTIMULib.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <time.h>
#include <stdlib.h>


#define ACK "ackfromunity"

struct packetData{
	RTFLOAT angle[3];
	RTFLOAT angleVel[3];
	RTFLOAT vel[3];
};

struct host_info{
	int sd;
	struct sockaddr_in recver;
};

int PORTNO;
float friction;
float accelFilter;

struct host_info initSocket();
int sndPkt(struct host_info *host, struct packetData* data);
void calcVel(RTVector3 *vel, RTVector3 *linearAccel, int sampleRate);


int main(int argc, char* argv[])
{

    if(argc >= 2){
        PORTNO = atoi(argv[1]);
    }
    else{
        PORTNO = 9002;
    }
    
    if(argc >= 3){
	friction = atof(argv[2]);
    }
    else{
	friction = 10;
    }
    if(argc >= 4){
	accelFilter = atof(argv[3]);
    }
    else{
	accelFilter = 0.3;
    }


        printf("%d\n",PORTNO);
	printf("%f\n",friction);

    int sampleCount = 0;
    int sampleRate = 0;
    uint64_t rateTimer;
    uint64_t displayTimer;
    uint64_t now;

    //---------------------------------------------- send

    struct packetData data;
    struct host_info host = initSocket();

    
    memset((char*)&data, '\0', sizeof(data));


    //---------------------------------------------- send


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

    RTVector3 deg;
    RTVector3 gravityAccel;
    RTVector3 linearAccel;

    RTVector3 velocity;
    velocity.zero();

    while (1) {
	//printf("%d\n", settings->m_fusionType);	

        usleep(imu->IMUGetPollInterval() * 1000);

        while (imu->IMURead())
	{
            RTIMU_DATA imuData = imu->getIMUData();
            sampleCount++;

            now = RTMath::currentUSecsSinceEpoch();


	    //------------------------------ linear Accel
	    deg = imuData.fusionPose;
	    gravityAccel.setX(-sin(deg.y()));
	    gravityAccel.setY(sin(deg.x())*cos(deg.y()));
	    gravityAccel.setZ(cos(deg.x())*cos(deg.y()));

	    linearAccel = imuData.accel;
	    linearAccel-=gravityAccel;
	    //----------------------------- 

            if ((now - displayTimer) > 100000) {
                //printf("1: Sample rate %d: %s\n", sampleRate, RTMath::displayDegrees("", imuData.fusionPose));	
		//printf("1: Sample rate %d: %s\n", sampleRate, RTMath::displayRadians("", linearAccel));	
		//printf("2: Sample rate %d: %s\n", sampleRate, RTMath::displayRadians("", imuData.compass));
		

		calcVel(&velocity, &linearAccel, sampleRate);
		    
		
		//printf("2: Sample rate %d: %s\n", sampleRate, RTMath::displayRadians("", velocity));	

		fflush(stdout);


		data.angle[0] = imuData.fusionPose.x() * RTMATH_RAD_TO_DEGREE;
		data.angle[1] = imuData.fusionPose.y() * RTMATH_RAD_TO_DEGREE;
		data.angle[2] = imuData.fusionPose.z() * RTMATH_RAD_TO_DEGREE;
		
		
		


		sndPkt(&host, &data);


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


void displayPasswd(){
	
}


int generatePasswd(){
	int ret = 0, i;
	srand(time(NULL));
	
	for(i=0;i<8;i++){
		ret*=10;
		ret+=(rand()%9);
	}

	return ret;

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

		
	if((sendto(host->sd, data, sizeof(*data), 0,
	(struct sockaddr*)&(host->recver), sizeof(host->recver))) == -1){
		perror("sendto");
		return -1;
	}

	return 0;

}

void calcVel(RTVector3 *vel, RTVector3 *linearAccel, int sampleRate){

	sampleRate = sampleRate == 0 ? 44:sampleRate;
		
	float eTime = 1.0/sampleRate;

	if(linearAccel->length() > accelFilter){
	linearAccel->setX(linearAccel->x()*eTime);
	linearAccel->setY(linearAccel->y()*eTime);
	linearAccel->setZ(linearAccel->z()*eTime);

	(*vel)+=(*linearAccel);

	}

	
	RTVector3 drag = (*vel);

	RTFLOAT mag = RTVector3::dotProduct((*vel), (*vel));


	

	if(mag > 0)
		drag.normalize();


	drag.setX(drag.x() * mag * friction * eTime);
	drag.setY(drag.y() * mag * friction * eTime);
	drag.setZ(drag.z() * mag * friction * eTime);

	printf("%f %f %f\n",linearAccel->x(), linearAccel->y(), linearAccel->z());

//	printf("acce: %f %f %f\ndrag: %f %f %f\nvel: %f %f %f\n\n",
//	linearAccel->x(),linearAccel->y(),linearAccel->z(),
//	drag.x(), drag.y(), drag.z(),
//	vel->x(), vel->y(), vel->z());

	(*vel)-= drag;
	
}
