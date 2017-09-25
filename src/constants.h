#ifndef ION_CONSTANTS_H_
#define ION_CONSTANTS_H_

#define TRUE (0==0)
#define FALSE (0!=0)

/**
 * Constants to be used throughout the project
 */
namespace Constants {
  // Used in calculations
  const float MSEC_TO_SEC = 1000000.0;
  const float E1 = 0.0001;
  
  // Console output
  const bool VERBOSE = FALSE;
  const bool DEBUG = FALSE;
  
  // Mode of operation
  const bool RADAR_ON = TRUE;
  const bool LASER_ON = TRUE;
}
#endif /* ION_CONSTANTS_H_ */
