#include <SFML/Graphics.hpp>
// #include "../msd-sim/msd-sim.hpp"
#include "Environment/src/WorldModel.hpp"
#include "Environment/src/Platform.hpp"

int main() {
    // Create a window
    sf::RenderWindow window(sf::VideoMode(800, 600), "SFML Circle");

    // Create a circle
    sf::CircleShape circle(50); // Radius of 50
    circle.setFillColor(sf::Color::Green); // Fill color
    circle.setPosition(375, 275); // Position the circle in the center

    // Main loop
    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        // Clear the window
        window.clear(sf::Color::Black);

        // Draw the circle
        window.draw(circle);

        // Display what has been drawn
        window.display();
    }

    return 0;
}
