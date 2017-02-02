# FicTrac
FicTrac provides ball tracking functionality for tethered fly experiments and was originally developed by Richard Moore.  He has generously provided us with the source code for FicTrac, but has requested that we not share it with others until he has a versioning system set up.  Please respect his wishes and keep all source local to the lab.

## fmf saver branch
If you wish to record FicTrac video in fmf format (ie raw video with time stamps) you are in luck!  A branch of the FicTrac project with the necessary code already exists.  Beware however, that due to some weird as of yet unresolved quirks in the code no guarentees are made regarding dropped frames if recording the full debug window in color.  Things seem to be fine, however, if the video saved is only a black and white subimage.  Regardless, follow the steps below to switch branches.

```bash
cd ~/src/FicTrac
git fetch
git checkout fmfsaver
git pull
cd fmfwrapper
git submodule update --init --recursive
make
```

## Useful Links
- General lab notes:  https://maimonlab-wiki.rockefeller.edu/index.php/FicTrac
- Installation:  https://maimonlab-wiki.rockefeller.edu/index.php/FicTrac_Installation
- FicTrac's Website:  http://rjdmoore.net/fictrac/
