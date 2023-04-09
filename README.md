# INTRODUCTION

This purpose of this package is calculate the error of destination point and actual position which robot can reach.

And data I want to store include 6 components, so I decided create my own message Error.msg

Namely, the error is subtraction between lastest point of /move_base_simple/goal and  lastest point of  /odom

 ![image](https://user-images.githubusercontent.com/105471622/230785467-eb6f2f58-2c0a-448b-a57a-590f2199644f.png)

https://www.youtube.com/watch?v=S2P0HQKm7xM
