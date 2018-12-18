
#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <ctype.h>
#include <unistd.h>

#include "apriltag.h"
#include "image_u8.h"
#include "tag36h11.h"

#include "zarray.h"
#include "getopt.h"

#include "mex.h"

int nrows;
int ncols; 

#ifdef __cplusplus
extern "C" {
#endif

double* getTag(char* path)
{

    apriltag_family_t *tf = NULL;
    tf = tag36h11_create();

    tf->black_border = 1;
    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);
    td->quad_decimate = 1.0;
    td->quad_sigma = 0.0;
    td->nthreads = 4;
    td->debug = 0;
    td->refine_decode = 0;
    td->refine_pose = 0;
    int quiet = 0;
    int maxiters = 1;
    const int hamm_hist_max = 10;


    int hamm_hist[hamm_hist_max];
    memset(hamm_hist, 0, sizeof(hamm_hist));
    image_u8_t *im = image_u8_create_from_pnm(path);
    if (im == NULL) {
	    printf("couldn't find %s\n", path);
	    return NULL;
    }

    zarray_t *detections = apriltag_detector_detect(td, im);

     nrows = detections->size;
     ncols = 9;
    
     if (nrows == 0)
	return NULL;

    double* output_matrix = (double *) malloc( nrows*ncols*sizeof(double));

    for (int i = 0; i < detections->size; i++) {
	    apriltag_detection_t *det;
	    zarray_get(detections, i, &det);


	    output_matrix[ncols*i+0] = det->id;
	    for(int j=0; j<4; j++)
	    {
		output_matrix[ncols*i+ 2*j +1] = det->p[j][0];
		output_matrix[ncols*i+ 2*j +2] = det->p[j][1];
	    }
	    
	    hamm_hist[det->hamming]++;

	    apriltag_detection_destroy(det);
    }

    

    zarray_destroy(detections);
    image_u8_destroy(im);

    // don't deallocate contents of inputs; those are the argv
    apriltag_detector_destroy(td);

    tag36h11_destroy(tf);
    return output_matrix;
}

void mexFunction(int nlhs, mxArray *plhs[],
                 int nrhs, const mxArray *prhs[])
{
    // path
    char* path;
    path =  mxArrayToString(prhs[0]);
    double* output_matrix = getTag(path);

    if(output_matrix!=NULL)
    {
	plhs[0] = mxCreateDoubleMatrix(nrows,ncols,mxREAL);
        double* outMatrix = mxGetPr(plhs[0]);

	for (int i = 0; i < nrows; i++) {
		for(int j=0; j<ncols; j++)
			outMatrix[i+j*nrows] = output_matrix[ncols*i+j];  
	}


    }
    else{
	plhs[0] = NULL;
   }	


}

#ifdef __cplusplus
}
#endif

