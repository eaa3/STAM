# Simple Tracking and Mapping - STAM

This is an opensource version of the code originally used for the level 3 of the [ISMAR 2015 Off-site tracking competition](http://ypcex.naist.jp/trakmark/tracking-competition/).
Our team "VoxarLabs" got first place in the level 3 of the competition (tracking and mapping).


### Dependencies ###

* OpenCV 2.4.x
* cvsba (https://www.uco.es/investiga/grupos/ava/node/39)
* LAPACK (https://pheiter.wordpress.com/2012/09/04/howto-installing-lapack-and-blas-on-mac-os/)


### Installing dependencies on OSX ###

* opencv 2.4.x
  * ``brew tap homebrew/science``
  * `` brew install opencv ``
* f2c
  * ``brew install --HEAD homebrew/boneyard/f2c``
* cvsba
  * Download, make and install csvba: https://www.uco.es/investiga/grupos/ava/node/39
  
### Installing dependencies on Ubuntu ###

* opencv 2.4.x 
  * Follow instructions at http://docs.opencv.org/2.4/doc/tutorials/introduction/linux_install/linux_install.html

* cvsba
  * Follow UNIX instructions at https://www.uco.es/investiga/grupos/ava/node/39


### Building STAM Linux/OSX ###

````
git clone https://github.com/eaa3/STAM.git

cd STAM
   
mkdir build 

cmake ..

make -j 
````

### Running STAM ###

#### Fetch competition data
````
cd data
source fetch_data.bash
````

#### Go to build directory
````
cd ../build
````

#### Run demo passing scenario number as argument 

Example for scenario 1:
````
./demo_level3 1
````

Inline-style: 
![alt text](https://github.com/eaa3/STAM/blob/master/gifs/ismar2015_tracking_S01.gif "Scenario 1")



