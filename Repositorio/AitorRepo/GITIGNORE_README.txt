##################################################
## -----------------------------------------------
## CÓMO NO HACER SEGUIMIENTO DE ARCHIVOS EN GIT 
## -----------------------------------------------
## LOLI PINEL DEL VALLE
## FEBRERO_2016 
##################################################


CREATE A LOCAL .gitignore
-------------------------

1.- Navegar a la terminal al repositorio de git, y de ahí a la carpeta donde queramos hacer el .gitignore local.
2.- Crear el fichero: $ touch gitignore
3.- Editar el fichero: $ nano gitignore


CREATE A GLOBAL .gitignore
--------------------------
1.- Muestra el contenido del fichero .gitignore: $ cat .git/info/exclude
2.- Editar el fichero: $ nano .git/info/exclude


EJEMPLO DE .gitignore
---------------------------
# Compiled source #
###################
*.com
*.class
*.dll
*.exe
*.o
*.so

# CMakeFiles #
##############
CMakeCache.txt
CMakeFiles
CMakeScripts
Makefile
cmake_install.cmake
install_manifest.txt

# Packages #
############
# it's better to unpack these files and commit the raw source
# git has its own built in compression methods
*.7z
*.dmg
*.gz
*.iso
*.jar
*.rar
*.tar
*.zip

# Logs and databases #
######################
*.log
*.sql
*.sqlite

# OS generated files #
######################
.DS_Store
.DS_Store?
._*
.Spotlight-V100
.Trashes
ehthumbs.db
Thumbs.db



GITIGNORE TEMPLATES IN: https://github.com/github/gitignore
----------------------------------------------------------- 
