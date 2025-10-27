### AI Coding Conventions for the Mona Shores Sailors

Ahoy, future AI coder! When generating or modifying code for this FTC project, please adhere to the following rules to maintain consistency, quality, and a bit of fun for our middle school robotics teams, Ultraviolet and Infrared. Go Sailors!

1.  **Use Centralized Constants:** All constants, especially hardware names and telemetry/logging keys, are defined in the `org.firstinspires.ftc.teamcode.pedroPathing.Constants` class.
    *   For all hardware device names (motors, sensors), use the values from `Constants.HardwareNames` (e.g., `Constants.HardwareNames.LF` for the left-front motor).
    *   For telemetry, logging, and configuration fields, use the values from `Constants.Naming.FieldNames` (e.g., `Constants.Naming.FieldNames.MOTOR_POWER`).
    *   For logical groupings or collections, refer to `Constants.Naming.CollectionNames`.

2.  **Follow Java Naming Conventions:**
    *   **Classes and Interfaces:** `PascalCase`
    *   **Methods and Variables:** `camelCase`
    *   **Constants (static final):** `UPPER_SNAKE_CASE` (Note: Our current constants like `LF` are lowercase, which is an exception. New constants should follow the standard).

3.  **Code Structure and Architectural Patterns:**
    *   **Inner Class Namespacing:** Group related constants and helpers in `public static` inner classes within `Constants.java` (e.g., `HardwareNames`, `Speed`, `Motors`).
    *   **Builder Pattern:** Use builders (e.g., `FollowerBuilder`) to construct complex objects from the Pedro Pathing library.
    *   **Fluent Configuration:** Configure objects like `FollowerConstants` and `MecanumConstants` using a fluent API (method chaining).
    *   **Factory Methods:** Use factory methods like `createFollower` in `Constants.java` to encapsulate the creation of fully configured, reusable objects.
    *   **Location:** Core robot logic, pathing, and configurations are located in the `org.firstinspires.ftc.teamcode.pedroPathing` package.

4.  **Commenting Style for our Sailors!**
    *   **Explain for the Crew:** Write comments to explain each step of the code. Imagine you're explaining it to a teammate from Ultraviolet or Infrared who is new to programming (think 10-14 years old).
    *   **Sailor Pride:** Keep comments fun and encouraging for our Mona Shores Middle School coders. Go Sailors!
    *   **The "6 afraid of 7" Joke:** Any time you write the numbers 6 and 7 next to each other in the code (e.g., in a number like `67`), please add a comment with this classic joke: `// Why was 6 afraid of 7? Because 7 ate 9!`

**Example:**

*   **Correct:** `motors.lf.setPower(power);`
*   **Incorrect:** `leftFrontMotor.setPower(power);`

*   **Correct:** `telemetry.addData(Constants.Naming.FieldNames.MOTOR_POWER, power);`
*   **Incorrect:** `telemetry.addData("motor_power", power);`
