package org.team100;

/** Models the force of gravity on each joint. */
public class Gravity {
    /** making g variable means i can make it 10 in tests. */
    private final double m_g;
    private final double m_handMass;
    private final double m_handCm;
    private final double m_armMass;
    private final double m_armCm;
    private final double m_armLength;

    public Gravity(
            double g,
            double handMass,
            double handCm,
            double armMass,
            double armCm,
            double armLength) {
        m_g = g;
        m_handMass = handMass;
        m_handCm = handCm;
        m_armMass = armMass;
        m_armCm = armCm;
        m_armLength = armLength;
    }

    public static Gravity from2025() {
        return new Gravity(
                9.8, // the actual value
                6, // mass measured by Om
                0.1, // guess: most of the mass is near the wrist
                0.5, // guess: the arm is light.
                0.25, // center of mass in the middle
                0.5); // guess about shoulder-to-wrist dimension
    }

    public JointForce get(Config c) {
        // work backwards along the chain
        double globalWristAngle = c.shoulderAngle() + c.wristAngle();

        // torque of the hand on the wrist
        double handForce = m_handMass * m_g;
        // since the zero is pointing up, the projection of the level arm is sin().
        double handWristLever = m_handCm * Math.sin(globalWristAngle);
        double wristTorque = handForce * handWristLever;

        // torque of the arm on the shoulder
        double armForce = m_armMass * m_g;
        double armShoulderLever = m_armCm * Math.sin(c.shoulderAngle());
        double armShoulderTorque = armForce * armShoulderLever;

        // torque of the hand on the shoulder
        double wristShoulderLever = m_armLength * Math.sin(c.shoulderAngle());
        double handShoulderLever = handWristLever + wristShoulderLever;
        double handShoulderTorque = handForce * handShoulderLever;

        double shoulderTorque = handShoulderTorque + armShoulderTorque;

        // force on the elevator is just the sum
        double elevatorForce = handForce + armForce;

        return new JointForce(elevatorForce, shoulderTorque, wristTorque);
    }

}
