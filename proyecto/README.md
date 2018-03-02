// Instrucciones para Instalar PCL (Ubuntu 16.04)

// Instalar Java
sudo add-apt-repository -y ppa:webupd8team/java && sudo apt update && sudo apt -y install oracle-java8-installer

// Pre Requisitos Universales

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



