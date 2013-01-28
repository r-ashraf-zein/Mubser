/*
 * ENPH459_NOD_Process.cpp
 *
 *  Created on: 2012-01-28
 *      Author: timshi
 */

#include "ENPH459.h"

int angle = 0; // kinect initial to be zero angle
int angle_enable = 0; // enable or disable angle motor
int stream_save_enable = 0; // enable to same the frames
int quit = 0; // quit
int ROI_enable = 0;
//int snap_save_enable=0; // save one frame // useless throw away now.
int switch_angle_value = 0; // initial position of switches
int switch_angle_enable_value = 0;
int switch_stream_save_enable_value = 0;
int switch_quit_value = 0;
int switch_ROI_enable_value = 0;
//int switch_snap_save_enableswitch_snap_save_enable_value=0;_value=0;

//int Current_Seed;
double Plane_AngleX, Plane_AngleY, Plane_Angle; //  calculate angle for each plane, based on the dy dx information

double Abs_Plane_Angle; // combine angle X,Y // compare Angle X and Angle Y individually
void switch_angle(int position) // opencv highgui trackbars
  	{
	angle = position - 30;
}
void switch_angle_enable(int position) {
	angle_enable = position;
}
void switch_stream_save_enable(int position) {
	stream_save_enable = position;
}
void switch_quit(int position) {
	quit = position;
}
void switch_ROI_enable(int position) {
	ROI_enable = position;
}
//void switch_snap_save_enable( int position )
//{
//	snap_save_enable = position;
//switch_snap_save_enable_value=0;
//}

int main(int argc, char **argv) {
	char showinfo[80]; // for display information on images,
	CvFont font; // for display information on images
	cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.7, 0.7, 0, 1, CV_AA); // for display information on images
	char filenamedepth[80]; // save file names
	char filenamedistance[80];
	char filenamergb[80];
	char filenamemap[80];
	char filenameoverhead[80];
	char filenamesaveall[80];
	FILE *fp = fopen("accdata.txt", "w"); // file to save accelerometer information
	NOD *nod = new NOD;

// create new structure. this is the main structure of the program. please look at header file for detail

	initialnod(0, nod); // initialize structure
	cvNamedWindow(WINDOW, 1);
// create main window, since trackbars will be attached to this window, it has to be create first.

	cvCreateTrackbar("Quit", WINDOW, &switch_quit_value, 1, switch_quit);
	//cvCreateTrackbar( "ROI", WINDOW, &switch_ROI_enable_value, 1, switch_ROI_enable );
	cvCreateTrackbar("Angle", WINDOW, &switch_angle_value, 60, switch_angle);
	cvCreateTrackbar("Enable_Angle", WINDOW, &switch_angle_enable_value, 1,
			switch_angle_enable);
	cvCreateTrackbar("Enable_Stream_Save", WINDOW,
			&switch_stream_save_enable_value, 1, switch_stream_save_enable);

	// the following data acquire steps will be repeated in the main loop below, for debug purpose, run them here first.
	freenect_sync_get_tilt_state(&nod->State, nod->Camera_Index); // get tilt state (accelerometer information)from kinect
	freenect_get_mks_accel(nod->State, &nod->Dx, &nod->Dy, &nod->Dz); // get indiviual dy dx dz.

	printf("%.3f,%.3f,%.3f\n", nod->Dx, nod->Dy, nod->Dz); // print out accelerometer information

	nod->RGB_Image = freenect_sync_get_rgb_cv(nod->Camera_Index); // get RGB image
	if (!nod->RGB_Image) {
		printf("Error: Kinect not connected?\n");
		return -1;
	}

	cvCvtColor(nod->RGB_Image, nod->RGB_Image, CV_RGB2BGR); // convert image color channel order.

	//   cvCopyImage(nod->RGB_Image,nod->General_Display);       //if want general display to be a color image
	nod->Depth_Image = freenect_sync_get_depth_cv(nod->Camera_Index); // get 16 bits Depth image

	while (quit == false) { // recall quit is a track bar that attached to window WINDOW
		cvZero(nod->General_Display); // swap General Display Image;

		cvZero(nod->Floor_Mapping_Image); // swap Floor_Mapping_Image;

		for (int i = 0; i < (nod->Cols * nod->Rows); i++) {

			nod->Overhead_View_Results[i].x = 0; // store the probability score here
												 // scale from 0% to 100%
												 // if no data, -1
			nod->Overhead_View_Results[i].y = 0; // store angle here, in radius
			nod->Overhead_View_Results[i].z = 0; // store distance here, in meter
		}

		nod->RGB_Image = freenect_sync_get_rgb_cv(nod->Camera_Index);
		cvCvtColor(nod->RGB_Image, nod->RGB_Image, CV_RGB2BGR);
		//cvCopyImage(nod->RGB_Image,nod->General_Display);  //if want general display to be a real scene based image.
		nod->Depth_Image = freenect_sync_get_depth_cv(nod->Camera_Index);
		kinect_angle(nod);

		if (ROI_enable) {
			nod->Display_Depth_Image = get_display_depth_image(
					nod->Depth_Image);
		} else {
			nod->Display_Depth_Image = GlViewColor(nod->Depth_Image);
		}

		if (angle_enable == 1)
		// adjust angle if enable, notice that the angle is determined by the accelerometer,
		// angle will re-adjust if kinect is moved
				{
			nod->Angle = angle;
			freenect_sync_set_tilt_degs(nod->Angle, nod->Camera_Index);

		}

		frame_analysis(nod);

		find_seed_roi(nod);
		if (ROI_enable) {
			draw_bplane_seed(nod);
		}
		find_floor(nod);

///////////////////////////

//double minz1 = nod->ROI_Avg[0],minz2 = nod->ROI_Avg[nod->ROI_nCols/3],minz3 = nod->ROI_Avg[nod->ROI_nCols*2/3],tmp;

		/*
		 double minz1 =3000,minz2 =3000,minz3 =3000,tmp;
		 for(int i=0;i<nod->ROI_nRows;i++)
		 {
		 for(int j=0;j< nod->ROI_nCols/3;j++)
		 {
		 Plane_AngleX=atan(-FOCAL*1.0/(((1090-nod->ROI_Avg[i*nod->ROI_nCols+j])/double(nod->ROI_DxDy[i*nod->ROI_nCols+j].x))-nod->ROI_Center	[i*nod->ROI_nCols+j].x+320)); // Balance angle


		 //!nod->ROI_bPlane[i*nod->ROI_nCols+j] &&
		 if((abs(Plane_AngleX*180/3.14+nod->Kinect_AngleX*180/3.14)>10) )
		 // The ROI in not in a plane and is not in a floor
		 {
		 //tmp = nod->ROI_Avg[i*nod->ROI_nCols+j];

		 int Start_Col = nod->ROI_Location[j].x ;
		 int Start_Row = nod->ROI_Location[j].y;

		 for (int n = Start_Col; n < Start_Col+ROI_SIZE; n++)
		 {
		 for (int m = Start_Row; m < Start_Row+ROI_SIZE; m++)
		 {

		 tmp =((short *)nod->Depth_Image->imageData)[nod->ROI_nCols*m+n];




		 if (tmp!=0 && tmp<minz1) minz1=tmp;
		 }
		 }

		 }
		 else tmp =3000;

		 }


		 //for(int j= nod->ROI_nCols/3;j< (nod->ROI_nCols*2)/3;j++)
		 for(int j=nod->ROI_nCols/3;j< nod->ROI_nCols*2/3;j++)
		 {
		 Plane_AngleX=atan(-FOCAL*1.0/(((1090-nod->ROI_Avg[i*nod->ROI_nCols+j])/double(nod->ROI_DxDy[i*nod->ROI_nCols+j].x))-nod->ROI_Center	[i*nod->ROI_nCols+j].x+320)); // Balance angle


		 //!nod->ROI_bPlane[i*nod->ROI_nCols+j] &&
		 if((abs(Plane_AngleX*180/3.14+nod->Kinect_AngleX*180/3.14)>10) )
		 // The ROI in not in a plane and is not in a floor
		 {
		 //tmp = nod->ROI_Avg[i*nod->ROI_nCols+j];

		 int Start_Col = nod->ROI_Location[j].x ;
		 int Start_Row = nod->ROI_Location[j].y;

		 for (int n = Start_Col; n < Start_Col+ROI_SIZE; n++)
		 {
		 for (int m = Start_Row; m < Start_Row+ROI_SIZE; m++)
		 {

		 tmp =((short *)nod->Depth_Image->imageData)[nod->ROI_nCols*m+n];




		 if (tmp!=0 && tmp<minz2) minz2=tmp;
		 }
		 }

		 }
		 else tmp =3000;

		 }


		 //for(int j= (nod->ROI_nCols*2)/3;j< nod->ROI_nCols;j++)
		 for(int j=0;j< nod->ROI_nCols/3;j++)
		 {
		 Plane_AngleX=atan(-FOCAL*1.0/(((1090-nod->ROI_Avg[i*nod->ROI_nCols+j])/double(nod->ROI_DxDy[i*nod->ROI_nCols+j].x))-nod->ROI_Center	[i*nod->ROI_nCols+j].x+320)); // Balance angle


		 //!nod->ROI_bPlane[i*nod->ROI_nCols+j] &&
		 if((abs(Plane_AngleX*180/3.14+nod->Kinect_AngleX*180/3.14)>10) )
		 // The ROI in not in a plane and is not in a floor
		 {
		 //tmp = nod->ROI_Avg[i*nod->ROI_nCols+j];

		 int Start_Col = nod->ROI_Location[j].x ;
		 int Start_Row = nod->ROI_Location[j].y;

		 for (int n = Start_Col; n < Start_Col+ROI_SIZE; n++)
		 {
		 for (int m = Start_Row; m < Start_Row+ROI_SIZE; m++)
		 {
		 tmp =((short *)nod->Depth_Image->imageData)[nod->ROI_nCols*m+n];




		 if (tmp!=0 && tmp<minz3) minz3=tmp;
		 }
		 }

		 }
		 else tmp =3000;

		 }
		 }


		 printf("left= %lf middle %lf right %lf \n",minz1,minz2,minz3);
		 */

		compute_results_angle_radius(nod);
		compute_results_score(nod);

		int minz1 = 3000, minz2 = 3000, minz3 = 3000, temp, z1_min_x, z1_min_y,
				z2_min_x, z2_min_y, z3_min_x, z3_min_y;
		for (int j = 0; j < nod->Rows; j += 3) {
			for (int i = 0; i < nod->Cols / 3; i += 10) {

				// check the difference between expect Plane Value
				// and actual Depth Value
					if (nod->Overhead_View_Results[nod->Cols * j + i].x < 0.4)
					{
					temp = ((short *) nod->Depth_Image->imageData)[(j* nod->Cols) + i];
					if (temp < minz1) {
						minz1 = temp;
						z1_min_x = i;
						z1_min_y = j;
					}
				}

			}

			for (int i = 214; i < (nod->Cols * 2) / 3; i += 10) {

				// check the difference between expect Plane Value
				// and actual Depth Value
				if (nod->Overhead_View_Results[nod->Cols * j + i].x < 0.4)
				{

					temp = ((short *) nod->Depth_Image->imageData)[(j* nod->Cols) + i];
					if (temp < minz2) {
						minz2 = temp;
						z2_min_x = i;
						z2_min_y = j;
					}

				}
			}

				for (int i = 427; i < nod->Cols; i += 10) {

					// check the difference between expect Plane Value
					// and actual Depth Value
				if (nod->Overhead_View_Results[nod->Cols * j + i].x < 0.4)
				{
						temp = ((short *) nod->Depth_Image->imageData)[(j* nod->Cols) + i];
						if (temp < minz3) {
							minz3 = temp;
							z3_min_x = i;
							z3_min_y = j;
						}

					}

				}

			}
			printf("left= %d middle %d right %d \n", minz1, minz2, minz3);

			cvCircle(nod->RGB_Image, cvPoint(z1_min_x, z1_min_y), 20,
					cvScalar(0xff, 0x00, 0x00));
			cvCircle(nod->RGB_Image, cvPoint(z2_min_x, z2_min_y), 20,
					cvScalar(0xff, 0x00, 0x00));
			cvCircle(nod->RGB_Image, cvPoint(z3_min_x, z3_min_y), 20,
					cvScalar(0xff, 0x00, 0x00));

			/*
			 int Depth_Value_1;
			 for(int j=0; j< nod->Rows; j+=10)
			 {
			 for (int i = 0; i < nod->Cols;i+=10)
			 {




			 Depth_Value_1=((short *)nod->Depth_Image->imageData)[nod->Cols*j+i];


			 printf("Depth is %d  \n",Depth_Value_1);
			 }
			 }
			 */

			map_direct_results(nod);
			opt_map_results(nod);

//	printf("%.3f,%.3f,%.3f,%.3f,%.3f\n",nod->Dx, nod->Dy, nod->Dz,nod->Kinect_AngleX*180/3.14,nod->Kinect_AngleY*180/3.14);
//printf("%.3f\n",nod->Mount_Height);
			sprintf(showinfo, "Height%.3f", nod->Mount_Height);
			cvPutText(nod->Floor_Mapping_Image, showinfo, cvPoint(20 * 3, 400),
					&font, cvScalar(0, 255, 0, 0));

			sprintf(showinfo, "Angle%.3f", nod->Kinect_AngleY * 180 / 3.14);
			cvPutText(nod->Floor_Mapping_Image, showinfo, cvPoint(20 * 3, 350),
					&font, cvScalar(0, 255, 0, 0));
			//printf("%.3f,%.3f\n", nod->Overhead_View_Results[153728].y, nod->Overhead_View_Results[153728].z*sin(-nod->Kinect_AngleY));
			//printf("%.3f,%.3f\n", nod->Overhead_View_Results[256128].y, nod->Overhead_View_Results[256128].z);

			cvCreateTrackbar("Quit", WINDOW, &switch_quit_value, 1,
					switch_quit);
			cvCreateTrackbar("ROI", "Depth", &switch_ROI_enable_value, 1,
					switch_ROI_enable);
			cvCreateTrackbar("Angle", WINDOW, &switch_angle_value, 60,
					switch_angle);
			cvCreateTrackbar("Enable_Angle", WINDOW, &switch_angle_enable_value,
					1, switch_angle_enable);
			cvCreateTrackbar("Enable_Stream_Save", WINDOW,
					&switch_stream_save_enable_value, 1,
					switch_stream_save_enable);
			cvShowImage(WINDOW, nod->Floor_Mapping_Image);
			cvShowImage("Depth", nod->Display_Depth_Image);
			cvShowImage("Floor detection", nod->General_Display);
			cvShowImage("RGB", nod->RGB_Image);
			cvWaitKey(15);

			if (stream_save_enable == 1) // save data if enable save.
					{
				fprintf(fp, "%04d,%.3f,%.3f,%.3f\n", nod->Record_Count, nod->Dx,
						nod->Dy, nod->Dz);
				sprintf(filenamedepth, "Depth_16_%04d.pgm", nod->Record_Count);
				sprintf(filenamergb, "RGB%04d.jpg", nod->Record_Count);
				sprintf(filenamemap, "Depth_8_%04d.jpg", nod->Record_Count);
				sprintf(filenamedistance, "FloorMap%04d.jpg",
						nod->Record_Count);
				sprintf(filenameoverhead, "OverHeadView%04d.jpg",
						nod->Record_Count);
				sprintf(filenamesaveall, "saveall%04d.jpg", nod->Record_Count);

				//	cvSaveImage(filenamedepth,nod->Depth_Image);
				cvSaveImage(filenamergb, nod->RGB_Image);
				cvSaveImage(filenamemap, nod->Display_Depth_Image);
				cvSaveImage(filenamedistance, nod->General_Display);
				cvSaveImage(filenameoverhead, nod->Floor_Mapping_Image);

				cvSetImageROI(nod->Save_All_Image, cvRect(0, 0, 640, 480));
				cvCopy(nod->RGB_Image, nod->Save_All_Image, NULL);
				cvResetImageROI(nod->Save_All_Image);

				cvSetImageROI(nod->Save_All_Image, cvRect(640, 0, 640, 480));
				cvCopy(nod->Display_Depth_Image, nod->Save_All_Image, NULL);
				cvResetImageROI(nod->Save_All_Image);

				cvSetImageROI(nod->Save_All_Image, cvRect(0, 480, 640, 480));
				cvCopy(nod->General_Display, nod->Save_All_Image, NULL);
				cvResetImageROI(nod->Save_All_Image);

				cvSetImageROI(nod->Save_All_Image, cvRect(640, 480, 640, 480));
				cvCopy(nod->Floor_Mapping_Image, nod->Save_All_Image, NULL);
				cvResetImageROI(nod->Save_All_Image);
				cvSaveImage(filenamesaveall, nod->Save_All_Image);

				nod->Record_Count++;
			}
			//nod->Record_Count++;
			//printf("%i\n",nod->Record_Count);
		}
		return 0;
	}

