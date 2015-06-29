FicTrac: a webcam-based method for generating fictive animal paths.
Copyright (C) 2011-2015 Richard Moore (rjdmoore@uqconnect.edu.au)

http://rjdmoore.net/fictrac

== LICENSE ==
This work is licensed under the Creative Commons
Attribution-NonCommercial-ShareAlike 3.0 Unported License.
To view a copy of this license, visit
http://creativecommons.org/licenses/by-nc-sa/3.0/

== WARRANTY ==
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY
KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE
USE OR OTHER DEALINGS IN THE SOFTWARE.

== TERMS OF USE ==
If you use this software and publish your data, you must cite:

  R. J. D. Moore, G. J. Taylor, A. C. Paulk, T. Pearson,
  B. van Swinderen, M. V. Srinivasan, "FicTrac: a visual method
  for tracking spherical motion and generating fictive animal paths",
  Journal of Neuroscience Methods, Volume 225, 30th March 2014,
  Pages 106-119, http://dx.doi.org/10.1016/j.jneumeth.2014.01.010.

Please subscribe to the FicTrac mailing list to be notified of
important updates. Subscribe at http://rjdmoore.net/fictrac/mail.html

Please also notify the author if you use this software, particularly
if you find it useful or you discover an issue - thanks!

== PREREQUISITES ==
This software uses libraries from the following projects:
    - OpenCV (http://opencv.org)
    - NLopt (http://ab-initio.mit.edu/nlopt/)
    - Cairo (http://www.cairographics.org)
These libraries are not distributed with this release but are
freely available as open-source projects.

== VERSION HISTORY & OPEN BUGS ==
Please see the VERSIONS_BUGS.txt file in this directory for version history
and open bugs.

== FILE DIRECTORY ==
This parent directory contains:
    - This README text file.
    - VERSIONS_BUGS text file, detailing version history and open bugs.
    - [docs] folder, which contains:
        - FicTrac.pdf, the FicTrac user manual.
        - Moore_etal-FicTrac.pdf, Journal of Neuroscience Methods preprint.
        - FicTrac.mp4, a short explanatory video showing sample output.
        - Several output file headers.
    - [sample] folder, which contains:
        - An example dataset containing all the input data and
          configuration files necessary to execute FicTrac.
        - How_to_run_sample.txt, which describes how to execute
          FicTrac on this sample data.
        - An example output video showing the expected results.
    - [old_versions] folder, which contains:
        - Several outdated executables (builds < 2014.07.14).
    - Several FicTrac executables:
        - FicTrac_ubuntu##.##_32bit, compiled for Ubuntu version ##.## (32-bit).
        - FicTrac_ubuntu##.##_64bit, compiled for Ubuntu version ##.## (64-bit).
        - FicTrac_ubuntu##.##_64bit-PGR, special build of FicTrac
          compiled with support for PointGrey USB3 cameras (requires
          PGR FlyCapture SDK to be installed on target machine).
    - run_fictrac.sh, script for batch processing multiple input
      video files (see user manual for how to use one config file for
      multiple input video files).
    - socket_client.py, an example python script showing how to
      connect to FicTrac (if do_socket_out is enabled) to receive
      output data in real time (see user manual for more information).
