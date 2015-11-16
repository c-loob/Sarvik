/*
NB!
liikumine toimub vektori abil kus: 
	esimene element tähistab liikumist y teljel(otse)
	teine liikumist x teljel(külgedele).
	kolmas roteerumist(positiivne = vastupäeva või vasakule, negatiivne = päripäeva või paremale)

liigutamiseks tuleb defineerida:
	float liigu[3] = { 0, 0, 1 };
	int max_speed = ?;

ning liikumise saab esile kutsuda:
	movement(liigu, max_speed);
*/

#include <opencv2\opencv.hpp>
#include <opencv2\highgui.hpp>
#include <opencv2\imgproc.hpp>
#include <iostream>
#include <vector>
#include <opencv\cv.h>
#include <stdio.h>
#include <time.h>//sleepcp - windows/posix sys-s? possible future error
#include <serial\serial.h>

using namespace cv;
using namespace std;

//global stuff
	vector< vector<Point> > contours_ball, contours_goal1, contours_goal2;
	vector< Vec4i > hierarchy_ball, hierarchy_goal;

	//sihtimise limiidid
	int vasak_limiit = 275;
	int parem_limiit = 365;

	//initial values for trackbars
	int G_lowH1 = 50;
	int G_highH1 = 70;
	int G_lowS1 = 50;
	int G_highS1 = 150;
	int G_lowV1 = 50;
	int G_highV1 = 200;

	int G_lowH2 = 72;
	int G_highH2 = 110;
	int G_lowS2 = 155;
	int G_highS2 = 237;
	int G_lowV2 = 50;
	int G_highV2 = 200;

	int B_lowH = 5;
	int B_highH = 25;
	int B_lowS = 80;
	int B_highS = 255;
	int B_lowV = 50;
	int B_highV = 255;

//headers
void sleepcp(int milliseconds);
void move_robot(int * kiirus);
void movement(float liigu[3]);
int ymarda(float a);
int * get_speed(float * joud);
float * move_vector(float liigu[3]);
int ymarda(float a);
void stop();
String receive(String port, int length);
void transmit(String port, String command);

void sleepcp(int milliseconds) // cross-platform sleep function
{
	clock_t time_end;
	time_end = clock() + milliseconds * CLOCKS_PER_SEC / 1000;
	while (clock() < time_end)
	{
	}
}

//get thersholded, eroded etc img
Mat preprocess(Mat frame, int lowH, int lowS, int lowV, int highH, int highS, int highV) {
	Mat thresh, HSV;
	cvtColor(frame, HSV, COLOR_BGR2HSV);//to HSV color space
	inRange(HSV, Scalar(lowH, lowS, lowV), Scalar(highH, highS, highV), thresh);

	erode(thresh, thresh, Mat(), Point(-1, -1), 2);
	dilate(thresh, thresh, Mat(), Point(-1, -1), 2);
	return thresh;
}

//get mass center of goal(mc of biggest contour)
Point2f process_goal(vector<vector<Point>> contours, Mat frame, Scalar varv) {
	float biggest_contour_area = 0;
	int biggest_contour_id = -1;
	vector<Point2f> mc_goal(contours.size());
	for (int i = 0; i < contours.size(); i++) {
		//drawContours(imgDrawing2, contours, i, Scalar(255, 0, 0), 1, 8, hierarchy_goal, 0, Point());
		float ctArea = contourArea(contours[i]);
		if (ctArea > biggest_contour_area) {
			biggest_contour_area = ctArea;
			biggest_contour_id = i;
		}
	}

	if (biggest_contour_id < 0) {
		Point2f temp;
		temp.x = -1;
		temp.y = -1;
		return temp;//väravat ei leitud
	}
	else {
		vector<Moments> mu_goal(contours.size());
		mu_goal[biggest_contour_id] = moments(contours[biggest_contour_id], false);

		mc_goal[biggest_contour_id] = Point2f(mu_goal[biggest_contour_id].m10 / mu_goal[biggest_contour_id].m00, mu_goal[biggest_contour_id].m01 / mu_goal[biggest_contour_id].m00);

		circle(frame, mc_goal[biggest_contour_id], 4, Scalar(255, 0, 0), -1, 8, 0);

		RotatedRect boundingBox = minAreaRect(contours[biggest_contour_id]);
		Point2f corners[4];
		boundingBox.points(corners);
		line(frame, corners[0], corners[1], varv);
		line(frame, corners[1], corners[2], varv);
		line(frame, corners[2], corners[3], varv);
		line(frame, corners[3], corners[0], varv);

		return mc_goal[biggest_contour_id];
	}
}

//väljastab ainult kõige suurema kontuuri, raadiuse
pair<Point2f, float> process_ball(vector<vector<Point>> contours, Mat frame) {
	vector<Moments> mu(contours.size());
	for (int i = 0; i < contours.size(); i++)
	{
		mu[i] = moments(contours[i], false);
	}
	//get mass centers
	vector<Point2f> mc(contours.size());
	if (contours.size() > 0) {
		for (int i = 0; i < contours.size(); i++)
		{
			mc[i] = Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
		}

		Mat imgDrawing = Mat::zeros(frame.size(), CV_8UC3);

		for (int i = 0; i < contours.size(); i++) {
			if (contourArea(contours[i])>100) {
				//drawContours(frame, contours_ball, i, Scalar(0, 0, 255), 2, 8, hierarchy_ball, 0, Point());
				//circle(frame, mc[i], 4, Scalar(255, 0, 0), -1, 8, 0);
				float radius(contours[i].size());
			}
		}
		float biggest_contour_area = 0;
		int biggest_contour_id = -1;

		for (int i = 0; i < contours.size(); i++) {
			//drawContours(imgDrawing2, contours, i, Scalar(255, 0, 0), 1, 8, hierarchy_goal, 0, Point());
			float ctArea = contourArea(contours[i]);
			if (ctArea > biggest_contour_area) {
				biggest_contour_area = ctArea;
				biggest_contour_id = i;
			}
		}
		float r = sqrt(biggest_contour_area / 3.1415);
		circle(frame, mc[biggest_contour_id], r, Scalar(255, 0, 0), -1, 8, 0);
		return make_pair(mc[biggest_contour_id], r);
	}
	else {
		//kui ühtegi palli ei leita, siis tagastatakse koordinaadid (-1, -1)
		Point2f temp;
		temp.x = -1;
		temp.y = -1;
		float temp2 = 0;
		return make_pair(temp, temp2);
	}
}

void transmit(String port, String command){
	try {
		serial::Serial my_serial(port, 19200, serial::Timeout::simpleTimeout(20));
		if (my_serial.isOpen()) {
			my_serial.write(command + "\n");
		}
	}
	catch (exception &e) {
		cerr << "unhandeled exception: " << e.what() << endl;

	}
}

String receive(String port, int length) {
	try {
		serial::Serial my_serial(port, 19200, serial::Timeout::simpleTimeout(200));
		if (my_serial.isOpen()) {
			String result;
			result = my_serial.read(length);//length - saadava str pikkus bytedes
			return result;
		}
		else {
			return "not open";// not open
		}
	}
	catch (exception &e) {
		cerr << "unhandeled exception: " << e.what() << endl;
		return "exception";
	}
}

void stop(){
	transmit("COM3", ("3:sd0"));//1. mootori kiirus
	transmit("COM3", ("2:sd0"));//2. mootori kiirus
	transmit("COM3", ("1:sd0"));//3. mootori kiirus
}

float * move_vector(float liigu[3]){//liigu[3] = liikumise vektor {x, y, w} w-nurkkiirendus, 0 kui ei taha pöörata
	//liikumise maatriks, et ei peaks iga kord arvutama
	float liikumine[3][3] = { { 0.57735, -0.33333, 0.33333 }, {-0.57735, -0.33333, 0.33333}, {0, 0.66667, 0.33333} };
	float f1, f2, f3, x, y, w;
	static float tagastus[3];//jõudude vektor mille pärast tagastame

	x = liigu[0];
	y = liigu[1];
	w = liigu[2];

	//arvutame iga mootori jõu
	f1 = liikumine[0][0] * x + liikumine[0][1] * y + liikumine[0][2] * w;
	f2 = liikumine[1][0] * x + liikumine[1][1] * y + liikumine[1][2] * w;
	f3 = liikumine[2][0] * x + liikumine[2][1] * y + liikumine[2][2] * w;

	tagastus[0] = f1;
	tagastus[1] = f2;
	tagastus[2] = f3;

	return tagastus;
}

int * get_speed(float * joud, int max_speed){
	static int tagastus[3];

	tagastus[0] = ymarda(joud[0]*max_speed);
	tagastus[1] = ymarda(joud[1]*max_speed);
	tagastus[2] = ymarda(joud[2]*max_speed);

	return tagastus;
}

int ymarda(float a){
	if (a > 0){
		int b = (int)(a + 0.5);
		return (int)b;
	}
	else if (a < 0){
		int b = (int)(a - 0.5);
		return (int)b;
	}
	else return 0;
}

void movement(float liigu[3], int max_speed){
	float *jouvektor;
	jouvektor = move_vector(liigu);

	int *kiirused;
	kiirused = get_speed(jouvektor, max_speed);

	move_robot(kiirused);
}

void move_robot(int * kiirus){
	//NB 3 ja 1 mootori id hetkel vahetuses, sellepärast antakse 1. mootori kiirus kolmandale jms
	transmit("COM3", ("3:sd" + to_string(kiirus[0])));//1. mootori kiirus
	transmit("COM3", ("2:sd" + to_string(kiirus[1])));//2. mootori kiirus
	transmit("COM3", ("1:sd" + to_string(kiirus[2])));//3. mootori kiirus

}



pair<Mat, Point2f> get_frame(VideoCapture cap){
	Mat frame, pall_thresh;
	cap >> frame;
	if (!cap.read(frame)) cout << "error reading frame" << endl;//check for error'

	//jagame pildi neljaks
	line(frame, Point(320, 0), Point(320, 480), Scalar(0, 0, 0), 2, 8, 0);
	line(frame, Point(0, 240), Point(640, 240), Scalar(0, 0, 0), 2, 8, 0);

	//joonistame "sihiku"
	line(frame, Point(vasak_limiit, 0), Point(vasak_limiit, 480), Scalar(255, 255, 255), 2, 8, 0);
	line(frame, Point(parem_limiit, 0), Point(parem_limiit, 480), Scalar(255, 255, 255), 2, 8, 0);

	Point2f mc_ball;//mass centers

	pall_thresh = preprocess(frame, B_lowH, B_lowS, B_lowV, B_highH, B_highS, B_highV);

	findContours(pall_thresh, contours_ball, hierarchy_ball, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

	pair<Point2f, float> result = process_ball(contours_ball, frame);
	mc_ball = result.first;
	float raadius = result.second;

	//arvutame palli !umbkaudse! kauguse kaamerast
	//päris palli suurus * käsitsi leitud fokaalpikkuse parameeter/palli diameeter pikslites
	float palli_kaugus = 2.5 * 1060 / (2 * raadius);
	putText(frame, (to_string(palli_kaugus) + "cm"), cvPoint(30, 60),
		FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0, 0, 255), 1, CV_AA);

	return make_pair(frame, mc_ball);
}

int main() {
	VideoCapture cap(0);//enter cam # or video location
	if (!cap.isOpened()) return -1; //check if succeeded
	
	cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);

	namedWindow("control_ball", WINDOW_AUTOSIZE);//trackbaride aken
	namedWindow("control_goal1", WINDOW_AUTOSIZE);//trackbaride aken
	namedWindow("control_goal2", WINDOW_AUTOSIZE);//trackbaride aken

	

	//trackbar creation
	createTrackbar("LowH", "control_ball", &B_lowH, 179);//hue
	createTrackbar("HighH", "control_ball", &B_highH, 179);
	createTrackbar("LowS", "control_ball", &B_lowS, 255);//saturation
	createTrackbar("HighS", "control_ball", &B_highS, 255);
	createTrackbar("LowV", "control_ball", &B_lowV, 255);//value
	createTrackbar("HighV", "control_ball", &B_highV, 255);

	createTrackbar("LowH", "control_goal1", &G_lowH1, 179);//hue
	createTrackbar("HighH", "control_goal1", &G_highH1, 179);
	createTrackbar("LowS", "control_goal1", &G_lowS1, 255);//saturation
	createTrackbar("HighS", "control_goal1", &G_highS1, 255);
	createTrackbar("LowV", "control_goal1", &G_lowV1, 255);//value
	createTrackbar("HighV", "control_goal1", &G_highV1, 255);

	createTrackbar("LowH", "control_goal2", &G_lowH2, 179);//hue
	createTrackbar("HighH", "control_goal2", &G_highH2, 179);
	createTrackbar("LowS", "control_goal2", &G_lowS2, 255);//saturation
	createTrackbar("HighS", "control_goal2", &G_highS2, 255);
	createTrackbar("LowV", "control_goal2", &G_lowV2, 255);//value
	createTrackbar("HighV", "control_goal2", &G_highV2, 255);

	vector< vector<Point> > contours_ball, contours_goal1, contours_goal2;
	vector< Vec4i > hierarchy_ball, hierarchy_goal;

	//communication const
	String port = "COM3";
	int baud = 19200;

	int state = 0;
	bool otse = false;

	//init FPS benchmark stuff
	//start/end times; fps; frame counter; seconds elapsed since start
	time_t start, end;
	double fps;
	int f_counter;
	double sec;
	//start the clock
	time(&start);

	//init counter
	int counter = 0;

	//eelmise suuna meelespidaja. 1 = vasakule, 2 = paremale, 3 = otse
	int suund = 0;
	int speed = 50;


	Mat frame, pall_thresh, v2rav_thresh1, v2rav_thresh2;//frame

	for (;;) {
		pair<Mat, Point2f> result = get_frame(cap);

		Mat frame = result.first;
		Point2f mc_ball = result.second;

		//TESTING AREA

		//receive commands
		String command;
		command = "t";
	
		String port = "COM4";

		
		serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(1000));
		if (my_serial.isOpen()){
			size_t bytes_wrote = my_serial.write(command);

			string result = my_serial.read(command.length());
			cout << "asdad" << result << endl;
		}
		else{
			cout << "errrrrrrrrrrrr" << endl;
		}
		//TESTING AREA


		//how much time passed
		time(&end);

		//calculate FPS --- hetkel ei tööta
		++f_counter;
		sec = difftime(end, start);
		fps = f_counter / sec;
		//print
		putText(frame, to_string(fps), cvPoint(30, 30),
			FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0, 0, 255), 1, CV_AA);
		
		

		
		//keera palli suunale;; EELDAB, et pall on vaateväljas!
		if (mc_ball.x != -1){
			if (counter == 4){//liigutame robotit iga 4 frame möödudes. AJUTINE!
				counter = 0;
				if (mc_ball.x < vasak_limiit){//pöörame vasakule(1)
					//stop();
					if (suund == 2){// kui viimane kord keerati paremale ja mindi pallist mööda, siis keerame vasakule, aga vaikselt
						int i = 0;
						while (1){
							//get new ball position
							pair<Mat, Point2f> result = get_frame(cap);

							if (i == 5){
								stop();
								i = 0;
							}

							frame = result.first;
							mc_ball = result.second;

							float liigu[3] = { 0, 0, 0.2 };
							movement(liigu, speed);
							//cout << "vaikselt vasak" << endl;
							imshow("orig2", frame);
							if (waitKey(30) >= 0) break;

							if ((mc_ball.x < parem_limiit) && (mc_ball.x >vasak_limiit)){
								//cout << "stop1" << endl;
								stop();
								break;//kui otsesuunas, siis breagime
							}
							i++;
						}
					}
					else {//muidu keerame rohkem vasakule(1)
						float liigu[3] = { 0.5, -0.1, 0.3 };//veits otse(vaja siduda palli kaugusega), kerge strafe vasakule ja pööre vasakule
						movement(liigu, speed);
						//cout << "kärmelt vasak" << endl;
					}

					suund = 1;
				}
				else if (mc_ball.x > parem_limiit){
					//stop();
					if (suund == 1){// kui viimane kord keerati vasakule ja mindi pallist mööda, siis keerame paremale, aga poole vähem
						int i = 0;
						while (1){
							pair<Mat, Point2f> result = get_frame(cap);

							if (i == 5){
								i = 0;
								stop();
							}

							frame = result.first;
							mc_ball = result.second;
							float liigu[3] = { 0, 0, -0.2 };
							movement(liigu, speed);
							//cout << "vaikselt parem" << endl;
							imshow("orig2", frame);
							if (waitKey(30) >= 0) break;
							if ((mc_ball.x > vasak_limiit) && (mc_ball.x < parem_limiit)){
								//cout << "stop2" << endl;
								stop();
								break;
							}
						}
					}
					else {
						//cout << "kärmelt parem" << endl;
						float liigu[3] = { 0.5, 0.1, -0.3 };//veits otse(vaja siduda palli kaugusega), kerge strafe paremale ja pööre paremale
						movement(liigu, speed);
					}
					suund = 2;
				}
				else {
					stop();
					float liigu[3] = { 1, 0, 0 };
					movement(liigu, speed);
					cout << "otse" << endl;
					suund = 3;
				}

			}
			else {
				counter += 1;
			}
			if (counter > 4) counter = 0;

		}
		

		imshow("orig", frame);
		if (waitKey(30) >= 0) break;//nupuvajutuse peale break

	}


	//destroyAllWindows();
	return 0;
}