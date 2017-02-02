# FicTrac
FicTrac provides ball tracking functionality for tethered fly experiments and was originally developed by Richard Moore.  He has generously provided us with the source code for FicTrac, but has requested that we not share it with others until he has a versioning system set up.  Please respect his wishes and keep all source local to the lab.

## NOTE:
The fmfsaver version of fictrac includes the submodule fmfwrapper.  By default git clones a reference to the submodule but does not actually download the files.  To avoid having to type `git submodule init` and `git submodule update` for each submodule, you can instead use 
```bash
git clone --recursive urltofictrac   
```
to automatically download everything.  Please note that the fmfsaver branch requires the boost libraries.  It is strongly recommended that you simlpy download and install them from source, compiling the libraries as needed.

## Useful Links
- General lab notes:  https://maimonlab-wiki.rockefeller.edu/index.php/FicTrac
- Installation:  https://maimonlab-wiki.rockefeller.edu/index.php/FicTrac_Installation
- FicTrac's Website:  http://rjdmoore.net/fictrac/
