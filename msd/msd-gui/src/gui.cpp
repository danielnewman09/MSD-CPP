#include <SFML/Graphics.hpp>
#include <SFML/Window/Keyboard.hpp>

#include "msd-sim/src/Environment/Platform.hpp"
#include "msd-sim/src/Environment/WorldModel.hpp"

int main()
{
  // create the window (remember: it's safer to create it in the main thread due
  // to OS limitations)
  sf::RenderWindow window(sf::VideoMode({800, 600}), "OpenGL");

  // Create a circle
  sf::CircleShape circle(50);             // Radius of 50
  circle.setFillColor(sf::Color::Green);  // Fill color
  circle.setPosition({375, 275});         // Position the circle in the center

  // Main loop
  while (window.isOpen())
  {
    // check all the window's events that were triggered since the last
    // iteration of the loop
    while (const std::optional event = window.pollEvent())
    {
      // "close requested" event: we close the window
      if (event->is<sf::Event::Closed>())
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
