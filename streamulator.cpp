/* Streamulator test platform
 * Original by Michiel van der Vlag, adapted by Matti Dreef
 */

#include "streamulator.h"
#include "final.h"


/* Load image from file into pixel stream
 *
 * filename - path to input image
 * stream   - output pixel stream
 * frames   - number of times to repeat the image
 */
void loadStream(const std::string &filename, pixel_stream &stream, int frames)
{
	cv::Mat srcImg;

	// Read input image
	srcImg = cv::imread(filename);
	if (srcImg.data == NULL)
	{
		std::cout << "##### Invalid input image, check the INPUT_IMG path #####" << std::endl;
		throw;
	}

	// A necessary conversion to obtain the right format
	cv::cvtColor(srcImg, srcImg, CV_BGR2RGBA);

	// Write input data
	for (int frame=0; frame < frames; frame++)
		cvMat2AXIvideo(srcImg, stream);
}


/* Process image stream
 *
 * src - source (input) stream
 * dst - destination (output) stream
 */
void processStream(pixel_stream &src, pixel_stream &dst)
{
	// Call stream processing function
//	while (!src.empty())
//		rgb2gray(src, dst);


    pixel_stream grey, gaussian, conv, suppress, thres;
    float angle;

    while (!src.empty()){
            greyscale(src, grey);
            gauss(grey, gaussian);
            angle = sobel_filter(gaussian, conv);
            suppression(angle, conv, suppress);
            threshold(suppress, thres);
            hystersis(thres, dst);

    }
}


/* Save raw pixel stream to file
 *
 * src      - input pixel stream
 * dst      - passthrough of pixel stream
 * filename - path to output image
 */
void saveRawStream(pixel_stream &src, pixel_stream &dst, const std::string &filename)
{
	std::vector<ap_uint<32>> pixeldata;
	pixel_data pixel;

	// Copy source data
	while(!src.empty())
	{
		src >> pixel;
		pixeldata.push_back(pixel.data | 0xFF000000);
		dst << pixel;
	}

	// Compute height
	int height = pixeldata.size() / WIDTH;

	cv::Mat dstImg(height, WIDTH, CV_8UC4, pixeldata.data());
	cv::cvtColor(dstImg, dstImg, CV_RGBA2BGR);
	cv::imwrite(filename, dstImg);
}


/* Save valid frame from pixel stream
 *
 * src        - input pixel stream
 * filename   - path to output image
 * skipframes - number of frames to skip before saving
 */
void saveValidStream(pixel_stream &src, const std::string &filename, int skipframes)
{
	static ap_uint<32> pixeldata[HEIGHT][WIDTH];
	pixel_data pixel;

	// Check if input stream has contents
	if (src.empty())
	{
		std::cout << "##### Stream to save is empty #####" << std::endl;
		throw;
	}

	// Find start of valid frame after frame skips
	int j = -1;
	do
	{
		src >> pixel;
		j++;

		if (src.empty())
		{
			std::cout << "##### No frame start found with pixel.user #####" << std::endl;
			throw;
		}
	} while (!(pixel.user == 1 && j >= skipframes*WIDTH*HEIGHT));

	int delay = j - skipframes*WIDTH*HEIGHT;
	std::cout << "HLS delay: " << delay;
	std::cout << "  (" << (float)delay/WIDTH << " lines)" << std::endl;

	// Read frame, keep track of lines missing pixel.last signal
	int missing_last = 0;

	for (int rows=0; rows < HEIGHT; rows++)
		for (int cols=0; cols < WIDTH; cols++)
		{
			pixeldata[rows][cols] = pixel.data | 0xFF000000; // OR with full alpha channel

			if (cols == WIDTH-1 && pixel.last != 1) // check pixel.last
				missing_last++;

			if (!src.empty()) // don't read from empty stream
				src >> pixel;
			else
				goto endloop;
		}
endloop:

	std::cout << "Number of lines missing pixel.last signal: " << missing_last << std::endl;

	// Purge remaining data in stream
	while (!src.empty())
		src >> pixel;

	// Save image by converting data array to matrix
	cv::Mat saveImg(HEIGHT, WIDTH, CV_8UC4, pixeldata);
	cv::cvtColor(saveImg, saveImg, CV_RGBA2BGR);
	cv::imwrite(filename, saveImg);
}


int main()
{
	pixel_stream srcStream;
	pixel_stream procStream;
	pixel_stream dstStream;


	loadStream(INPUT_IMG, srcStream, FRAMES+2);

	processStream(srcStream, procStream);

	saveRawStream(procStream, dstStream, RAW_OUTPUT_IMG);

	saveValidStream(dstStream, OUTPUT_IMG, FRAMES);


	return 0;
}

