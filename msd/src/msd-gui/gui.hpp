#ifndef HELLO_WORLD_GUI_HPP
#define HELLO_WORLD_GUI_HPP

#include <SFML/Graphics.hpp>

class HelloWorldGui {
public:
    HelloWorldGui();
    void run();

private:
    sf::RenderWindow window;
    sf::Font font;
    sf::Text text;
};

#endif // HELLO_WORLD_GUI_HPP
