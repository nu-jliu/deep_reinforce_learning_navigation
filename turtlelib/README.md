# Turtlelib Library
A library for handling transformations in SE(2) and other turtlebot-related math.

# Components
- geometry2d - Handles 2D geometry primitives
- se2d - Handles 2D rigid body transformations
- frame_main - Perform some rigid body computations based on user input

# Conceptual Questions
1. If you needed to be able to ~normalize~ Vector2D objects (i.e., find the unit vector in the direction of a given Vector2D):
   - Propose three different designs for implementing the ~normalize~ functionality
    1. Add a generic function that takes an input of Vector2D, and output the normalized Vector2D
    2. Pass py reference, that normalize function take the input of the Vector2D and normalize it then modify it as the modified version of the Vector2D
    3. Make a operator operates on the Vector2D and returns a normalized vector.

   - Discuss the pros and cons of each proposed method, in light of the C++ Core Guidelines.
   1. **Pro:**: The variable is multable, so that do not need to worry about the input being modified. **Con**: In sometime maybe uses extra memory spaces.
   2. **Pro**: Since passing by reference, so that the only one memory space is used for contaning the variable, which conserves the memory spaces. **Con**: Since it is passing by reference, so that the input value will be modified, thus if would like to conserve the original one, need to make a copy before calling the function.
   3. **Pro**: It is easy to call. **Con**: sometimes it might be confusing to use it.

   - Which of the methods would you implement and why?
   I would like to implement the first one since we have a lot memory spaces, so that do don't really care about that amount of memory spaces being wasted. 

2. What is the difference between a class and a struct in C++?
   - The class coresponding an object, which can contain private and public members, each member can be either class method or attribute, which private members can only be accessed and modified within a class.
   - The struct is just a conbination of different data, it does not contain any methods, and all data can be accessed and modified anywhere.

<!-- TODO: Complete questions -->

3. Why is Vector2D a struct and Transform2D a Class (refer to at least 2 specific C++ core guidelines in your answer)?
 - Since the Vector2D does not need to perform any operation on itself, or perform any operation on others so that it only need to contain the data required that does not need to have any member methods. 


4. Why are some of the constructors in Transform2D explicit (refer to a specific C++ core guideline in your answer)?


5. Why is Transform2D::inv() declared const while Transform2D::operator*=() is not?
   - Refer to [[https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#con-constants-and-immutability][C++ Core Guidelines (Constants and Immutability)]] in your answer