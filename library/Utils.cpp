#include "Utils.h"

#include <boost/shared_ptr.hpp>

#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <deque>
#include <cmath>
#include <stdint.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <string.h>
#include <sched.h>

void Utils::SET_PROCESS_PRIORITY(int priority)
{
	setpriority(PRIO_PROCESS, 0, priority);
	int p = getpriority(PRIO_PROCESS, 0);
	printf("%s: Current process priority set to %d.\n", __func__, p);
}

bool Utils::READ_FILE_TO_STRING(const std::string fn, std::string& ret)
{
	std::stringstream buffer;
	std::ifstream file;
	file.open(fn.c_str(),std::ifstream::in);
	if( !file.is_open() ) {
		fprintf(stderr, "%s: Error, unable to open file (%s)!\n", __func__, fn.c_str());
		return false;
	}
	buffer << file.rdbuf();
	file.close();
	ret = buffer.str().substr(0);
	return true;
}

std::string Utils::GET_TOKEN(std::string& str, std::string dlim)
{
    std::string::size_type lastPos = str.find_first_not_of(dlim,0);
    std::string::size_type pos = str.find_first_of(dlim,lastPos);
    
    if( lastPos == std::string::npos ) { return std::string(""); }		// no non-delimiters found
    if( pos == std::string::npos ) {									// no trailing delimiters found
    	str = str.substr(lastPos);
    	return str;
    }
    
    std::string token = str.substr(lastPos,pos-lastPos);
    lastPos = str.find_first_not_of(dlim,pos);
    if( lastPos == std::string::npos ) {	// no non-delimiters found
    	str = std::string("");
    } else {
	    str = str.substr(lastPos);
	}
    return token;
}

void Utils::TOKENISE(const std::string& str, std::deque<std::string>& tokens, std::string dlim)
{
	// Skip delimiters at beginning.
    std::string::size_type lastPos = str.find_first_not_of(dlim,0);
    // Find first "non-delimiter".
    std::string::size_type pos     = str.find_first_of(dlim,lastPos);

    tokens.clear();
    while( std::string::npos != pos || std::string::npos != lastPos ) {
        // Found a token, add it to the vector.
        tokens.push_back(str.substr(lastPos,pos-lastPos));
        // Skip delimiters.  Note the "not_of"
        lastPos = str.find_first_not_of(dlim,pos);
        // Find next "non-delimiter"
        pos = str.find_first_of(dlim,lastPos);
    }
}

double Utils::STR2NUM(std::string str)
{
	double tmp;
	std::stringstream buf;
	buf << str.c_str();
	buf >> tmp;
	return tmp;
}

std::string Utils::NUM2STR(double num)
{
	std::stringstream str;
	str << num;
	return str.str();
}

static bool seeded = false;

/**
 * Generates uniformly distributed random floating point number in the range [min,max]
 */
double Utils::GEN_RAND_DBL(double min, double max)
{
	if( !seeded ) {
		srand(time(NULL));
		seeded = true;
	}
	if( min > max ) {
		double tmp = min;
		min = max;
		max = tmp;
	}
	return (max-min)*(rand()/double(RAND_MAX))+min;
}

/**
 * Generates guassian distributed random floating point number with mean and sigma
 */
double Utils::GEN_RAND_GSN(double sigma, double mean)
{
	if( !seeded ) {
		srand(time(NULL));
		seeded = true;
	}
	
	static double x = 0;
	static double y = 0;
	static double s = 0;
	double ret = 0;
	static int ncalls = 0;
	if( (++ncalls)%2 ) {
		// box-muller transform
		double r2 = 0;
		do {

			// choose x,y in uniform square
			x = GEN_RAND_DBL(-1,1);
			y = GEN_RAND_DBL(-1,1);

			// see if it is within the unit circle
			r2 = x*x+y*y;
		} while( (r2 > 1.0) || (r2 == 0) );
		s = sqrt(-2.0*log(r2)/r2);
		ret = sigma*x*s+mean;
	} else {
		ret = sigma*y*s+mean;
	}
	return ret;
}

double Utils::GET_CLOCK()
{
	timeval current;
	gettimeofday(&current, NULL);
	return current.tv_sec+current.tv_usec*1e-6;
}

std::string Utils::GET_DATE_STRING()
{
	time_t rawtime;
	struct tm * timeinfo;
	char buffer [80];

	time(&rawtime);
	timeinfo = localtime(&rawtime);

	strftime(buffer, 80, "%Y.%m.%d", timeinfo);
	return std::string(buffer);
}
