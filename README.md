# Box2D top-down car demo
-------------------------------

[Box2D v2.4.1](https://github.com/erincatto/box2d/releases/tag/v2.4.1) was used to create the example and is meant to be used within the Box2D Testbed. 

The demo is a basic example of top-down car physics and the linear algebra involved in 2D top-down car controls.

![Gif missing](https://media.giphy.com/media/xd17dxskxZkAquKvqR/giphy.gif)


Additionally, the demo also contains an application of the Wandering and Collision Avoidance steering behaviours, from Craig Reynolds' [paper](https://www.red3d.com/cwr/steer/gdc99/) form the late 90s. The AI steering example here is a bit different from the general one in the paper, as most of the calculations are in terms of the car controls and forces used here.

![Gif missing](https://media.giphy.com/media/W67WftD0f53rKGoEHb/giphy.gif)


Lastly, the demo also generates looping race tracks on every run. The procedural generation is seed-based, so if interested in repeating some tracks - the seed is shown on screen and can be set in code (I did not bother creating UI to input a seed, as all the generation is done within constructors). 

To generate the tracks themselves, the code applies a [Convex Hull](https://en.wikipedia.org/wiki/Convex_hull) algorithm - the [Graham Scan](https://en.wikipedia.org/wiki/Graham_scan). Shout-out to the StableSort Youtube channel for the best tutorial on the subject found online - see [here](https://www.youtube.com/watch?v=B2AJoQSZf4M&t=1s).



##### Note: 
-------------------------------
I created this project with learning purposes mostly - to revise some core linear algebra, as well as learn more about Box2D and physics engines in general. I don't intend to expand and maintain this in the long term. 

Do, however, feel free to raise any issues if any of the example is outright wrong, misleading, or broken - I want to make sure it's a good example. 

Alternatively, I can be found [@elhubanov](https://twitter.com/elhubanov) on twitter - feel free to DM there as well if any issues, or just to chat.


Usage/Building
-------------------------------
* Get & Build [Box2D v2.4.1](https://github.com/erincatto/box2d/releases/tag/v2.4.1)
* Add file as a test to the testbed project & build with the respective build system
