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









