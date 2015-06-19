# Arduino Library for HG7881 (L9110) H-bridges to works with GCodeCNCDemo

This is a Stepper library for use with HG7881 or L9110 H-bridges.

Because these chips have poor thermal management and thus overheat easily.

This library can handle the problem, they are not put into idle mode after stepping (and keep needlessly drawing huge amounts of current).

Idle mode is when inputs A and B are either both HIGH or both LOW. This wrapper drives them both LOW after stepping.

