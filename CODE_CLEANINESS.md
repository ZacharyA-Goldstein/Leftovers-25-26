

## 1. Avoid Unnecessary Nesting.

Why? Makes it easier to follow logic in the code.

How? Inverse clauses to get rid of nesting.
### BAD
```java
if (something) {
  // a ton of code
}
```
### GOOD
```java
if (!something) {
  return // or continue in loops
}
// a ton of code
```
3. Don't repeat code. Abstract repeated
4. Extract code into functions for better readability.
    to have better naming and make it easiler to understand without looking to much into it.
### BAD
```java
double velocityGoal = Math.signum(distance) * Math.sqrt(-2 * decleration * Math,abs(distance))
```
### GOOD
```java
double velocityGoal = Kinematics.getVelocityToStopWithDeceleration(deceleration, distance)
```
6. Every class/function should only do one thing
7. Follow obvious naming conventions such as `ClassesPasalCase`, `variablesCamelCase`
8. Variables, class names, and package names should be short, descriptive, and specific names without abbreviating anything
### BAD
```java
Point p = new Point(5, 6)
```
### GOOD
```java
Point endPoint = new Point(5, 6)
```
8. Docstrings should give more infomation than just the name of the method itself.
