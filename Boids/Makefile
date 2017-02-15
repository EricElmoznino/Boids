OBJS = Boids.o imgui_impl_glut.o imgui.o imgui_draw.o
CXXFLAGS = -O4 -g -fopenmp -L./lib -l3ds -I./include -lGL -lglut -lGLU -lm 


Boids: $(OBJS)
	g++-6 -Wno-deprecated -o $@ $^ -L./lib -l3ds  -framework OpenGL -framework GLUT

%.o: %.cpp
	g++-6 -Wno-deprecated -c $(CXXFLAGS) -o $@ $<

clean:
	rm *.o Boids
