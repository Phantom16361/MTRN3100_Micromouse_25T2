All cpp/hpp files now contain header sections that help with version control.

If you make any changes, please update the changelog, update date and add your name after the update date.

It is _**VERY**_ important to keep track of which version of a document we are using so we don't get things mixed up. **(INCLUDING YOUR main.ino)**

For files with "Do not edit unless discussed" in the header, it is not a good idea to introduce any change to them unless you know exactly what you are trying to do and you are sure it will not mess with other's code.

`Example:
/**************************************************************
 *  File         : CubicTrajectory.cpp
 *  Author       : Jason E Tomczyk
 *  Description  : Generates cubic time-based trajectories for
 *                 smooth motion planning from an initial state
 *                 to a target state over a fixed duration.
 * 
 *  Version      : 1.0
 *  Created On   : 2025-07-16
 *  Last Updated : 2025-07-16
 * 
 *  Changelog:
 *    - [v1.0] Implemented cubic coefficient solver and accessors
 *             for position, velocity, and acceleration profiles.
 *************************************************************/`
