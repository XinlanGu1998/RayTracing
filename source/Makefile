RayTracing :
	g++ -o RayTracing -std=c++11 main.cpp Scene.cpp Renderer.cpp

clean :
	rm RayTracing

test : 
	make clean
	make RayTracing
	./RayTracing
tmp :
	g++ -o RayTracing2 -std=c++11 main.cpp Scene.cpp Renderer.cpp
	./RayTracing2