#include <stdio.h>
#include <GLFW/glfw3.h>

void windowSizeCB(GLFWwindow *window, int width, int height);

void mouseButtonCB(GLFWwindow *window, int button, int action, int mods);

void mousePosCB(GLFWwindow *window, double x, double y);

void mouseScrollCB(GLFWwindow *window, double x, double y);

void keyFunCB(GLFWwindow* window, int key, int scancode, int action, int mods);

void charFunCB(GLFWwindow* window, unsigned int charInfo);

void dropCB(GLFWwindow *window, int num, const char **paths);


int main() {
    GLFWwindow* window;

    // Initialize the library
    if (!glfwInit()) return -1;

    // Create a windowed mode window and its OpenGL context
    window = glfwCreateWindow(640, 480, "Hello World", NULL, NULL);
    if (!window) {
        glfwTerminate();
        return -1;
    }

    // Set callback function
    {
        glfwSetWindowSizeCallback(window, windowSizeCB);

        glfwSetMouseButtonCallback(window, mouseButtonCB);
        glfwSetCursorPosCallback(window, mousePosCB);
        glfwSetScrollCallback(window, mouseScrollCB);

        glfwSetKeyCallback(window, keyFunCB);
        glfwSetCharCallback(window, charFunCB);

        glfwSetDropCallback(window, dropCB);
    }

    // Make the window's context current
    glfwMakeContextCurrent(window);

    // Loop until the user closes the window
    while (!glfwWindowShouldClose(window)) {
        // Render here */
        glClear(GL_COLOR_BUFFER_BIT);

        // Swap front and back buffers
        glfwSwapBuffers(window);

        // Poll for and process events
        glfwPollEvents();
    }

    glfwTerminate();
    return 0;
}

void windowSizeCB(GLFWwindow *window, int width, int height) {
    printf("windowSizeCB %d %d\n", width, height);
}

void mouseButtonCB(GLFWwindow *window, int button, int action, int mods) {
    printf("mouseButtonCB %d %d %d\n", button, action, mods);
}

void mousePosCB(GLFWwindow *window, double x, double y) {
    printf("mousePosCB %.1lf %.1lf\n", x, y);
}

void mouseScrollCB(GLFWwindow *window, double x, double y) {
    printf("mouseScrollCB %.1lf %.1lf\n", x, y);
}

void keyFunCB(GLFWwindow* window, int key, int scancode, int action, int mods) {
    printf("keyFunCB %d %d %d %d\n", key, scancode, action, mods);
}

void charFunCB(GLFWwindow* window, unsigned int charInfo) {
    printf("charFunCB %d\n", charInfo);
}

void dropCB(GLFWwindow *window, int num, const char **paths) {
    printf("dropCB %d\n", num);
    for (int i = 0; i < num; i++) {
        printf("%s\n", paths[i]);
    }
}
