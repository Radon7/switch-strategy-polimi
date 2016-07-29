mv dist/MRESim_4.0.jar MRESim_4.0.jar
java -jar -Xdebug -Xrunjdwp:transport=dt_socket,server=y,address=8888,suspend=n MRESim_4.0.jar
