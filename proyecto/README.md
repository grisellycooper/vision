// Instrucciones para Instalar PCL (Ubuntu 16.04)

// Instalar Java
sudo add-apt-repository -y ppa:webupd8team/java && sudo apt update && sudo apt -y install oracle-java8-installer

// Pre Requisitos Universales
sudo apt -y install g++ cmake cmake-gui doxygen mpi-default-dev openmpi-bin openmpi-common libusb-1.0-0-dev libqhull* libusb-dev libgtest-dev
sudo apt -y install git-core freeglut3-dev pkg-config build-essential libxmu-dev libxi-dev libphonon-dev libphonon-dev phonon-backend-gstreamer
sudo apt -y install phonon-backend-vlc graphviz mono-complete qt-sdk libflann-dev

// Algunas Dependencias
sudo apt-get install libeigen3-dev
sudo apt-get install libflann-dev

sudo apt -y install libflann1.8 libboost1.58-all-dev

// Instalar PCL usando Aptitude (apps por defecto)

 sudo apt install libpcl-dev

// A veces sale error de cannot find -lvtkproj4
// Instalar Python VTK
sudo apt update
sudo apt install python-vtk

// Aun a pesar a veces no agarra la libreria asi q usar
sudo apt install libproj-dev

sudo ln -s /usr/lib/x86_64-linux-gnu/libvtkCommonCore-6.2.so /usr/lib/libvtkproj4.so

Ejemplo para probar el Correcto Funcionamiento
en base a 2 archivos encontrados en la carpeta Test

ir a la Carpeta Test encontrada aqui
cd Test
mkdir build && cd build
cmake ..
make

ejecutar el programa
./pcl_visualizer_demo (con alguna opcion)

// LA CARPETA QUE CONTIENE EL EJEMPLO EN VFH ES TEST2

// PARA COMPILAR

mkdir build && cd build
cmake ..

luego correr

./build_tree data/  --> Para generar un Arbol KdTree para neareste neighbors

podemos correr un solo ejemplo

./nearest_neighbors -k 16 -thresh 50 data/000.580.67/1258730231333_cluster_0_nxyz_vfh.pcd

o un conjunto a traves de un ejemplo de archivo bash

vi test.sh

---test.sh---
#!/bin/bash

# Example directory containing _vfh.pcd files
DATA=data

# Inlier distance threshold
thresh=50

# Get the closest K nearest neighbors
k=16

for i in `find $DATA -type d -name "*"`
do
  echo $i
  for j in `find $i -type f \( -iname "*cluster*_vfh.pcd" \) | sort -R`
  do
    echo $j
    ./nearest_neighbors -k $k -thresh $thresh $j -cam "0.403137,0.868471/0,0,0/-0.0932051,-0.201608,-0.518939/-0.00471487,-0.931831,0.362863/1464,764/6,72"
  done
done
---test.sh---

bash test.sh

// OJO TENER LA CARPETA DATA EN LA CARPETA /build
// Ademas si se cuenta con anaconda.. Desactivar el PATH que se escribe automaticamente al instalar

si sale echo $PATH  -- > /something/anaconda3/lib..

cambiar y actualizar el ~/.bashrc y eliminar dicho PATH

luego borrar el build y volver a compilar (tb reiniciar la terminal)
actualizar con $:source ~/.bashrc


# Kinect 1 SET UP

Pre-Reqs
1. Opencv(2.4.13)
instalar con cmake -D WITH_OPENNI=ON ..

2. OPENNI ver 1.5
Building OpenNI:
		1) Go into the directory: "Platform/Linux/CreateRedist".
		   Run the script: "./RedistMaker".
		   This will compile everything and create a redist package in the "Platform/Linux/Redist" directory.
		   It will also create a distribution in the "Platform/Linux/CreateRedist/Final" directory.
		2) Go into the directory: "Platform/Linux/Redist".
		   Run the script: "sudo ./install.sh" (needs to run as root)

  		   The install script copies key files to the following location:
		       Libs into: /usr/lib
		       Bins into: /usr/bin
		       Includes into: /usr/include/ni
		       Config files into: /var/lib/ni
			
		To build the package manually, you can run "make" in the "Platform\Linux\Build" directory.
		If you wish to build the Mono wrappers, also run "make mono_wrapper" and "make mono_samples".

3. Sensor Kinect Driver
Linux:
	Requirements:
		1) GCC 4.x
		   From: http://gcc.gnu.org/releases.html
		   Or via apt:
		   sudo apt-get install g++
		2) Python 2.6+/3.x
		   From: http://www.python.org/download/
		   Or via apt:
		   sudo apt-get install python
		3) OpenNI 1.5.x.x
		   From: http://www.openni.org/Downloads/OpenNIModules.aspx
		   
	Building Sensor:
		1) Go into the directory: "Platform/Linux/CreateRedist".
		   Run the script: "./RedistMaker".
		   This will compile everything and create a redist package in the "Platform/Linux/Redist" directory.
		   It will also create a distribution in the "Platform/Linux/CreateRedist/Final" directory.
		2) Go into the directory: "Platform/Linux/Redist".
		   Run the script: "sudo ./install.sh" (needs to run as root)

  		   The install script copies key files to the following location:
		       Libs into: /usr/lib
		       Bins into: /usr/bin
		       Config files into: /usr/etc/primesense
		       USB rules into: /etc/udev/rules.d 
		       Logs will be created in: /var/log/primesense
			
		To build the package manually, you can run "make" in the "Platform\Linux\Build" directory.
		
		Important: Please note that even though the directory is called Linux, you can also use it to compile it for 64-bit targets and pretty much any other linux based environment.	

COMPILAR
	g++ -std=c++11 main.cpp `pkg-config opencv --cflags --libs` -o app && sudo ./app







# referencias
https://github.com/PointCloudLibrary/pcl/blob/master/doc/tutorials/content/vfh_estimation.rst
https://www.slideshare.net/antiw/cvpr2010-open-source-vision-software-intro-and-training-part-viii-point-cloud-library-rusu-unknown-2010



