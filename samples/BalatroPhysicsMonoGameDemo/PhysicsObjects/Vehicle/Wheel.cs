﻿using BalatroPhysics;
using BalatroPhysics.Collision;
using BalatroPhysics.Dynamics;
using BalatroPhysics.LinearMath;
using System;

namespace BalatroPhysicsDemo
{

    #region Copyright Notice - Wheel based on Jiglib
    /*
    Copyright (c) 2007 Danny Chapman 
    http://www.rowlhouse.co.uk

    This software is provided 'as-is', without any express or implied
    warranty. In no event will the authors be held liable for any damages
    arising from the use of this software.

    Permission is granted to anyone to use this software for any purpose,
    including commercial applications, and to alter it and redistribute it
    freely, subject to the following restrictions:

    1. The origin of this software must not be misrepresented; you must not
    claim that you wrote the original software. If you use this software
    in a product, an acknowledgment in the product documentation would be
    appreciated but is not required.

    2. Altered source versions must be plainly marked as such, and must not be
    misrepresented as being the original software.

    3. This notice may not be removed or altered from any source
    distribution.

    */
    #endregion


    /// <summary>
    /// A wheel which adds drive forces to a body.
    /// Can be used to create a vehicle.
    /// </summary>
    public class Wheel
    {
        private World world;

        private RigidBody car;

        private float displacement, upSpeed, lastDisplacement;
        private bool lastOnFloor;
        private float driveTorque;

        private float angVel;

        /// used to estimate the friction
        private float angVelForGrip;
        private float torque;

        private RaycastCallback raycast;

        /// <summary>
        /// Sets or gets the current steering angle of
        /// the wheel in degrees.
        /// </summary>
        public float SteerAngle { get; set; }

        /// <summary>
        /// Gets the current rotation of the wheel in degrees.
        /// </summary>
        public float WheelRotation { get; private set; }

        /// <summary>
        /// The damping factor of the supension spring.
        /// </summary>
        public float Damping { get; set; }

        /// <summary>
        /// The supension spring.
        /// </summary>
        public float Spring { get; set; }

        /// <summary>
        /// Inertia of the wheel.
        /// </summary>
        public float Inertia { get; set; }

        /// <summary>
        /// The wheel radius.
        /// </summary>
        public float Radius { get; set; }

        /// <summary>
        /// The friction of the car in the side direction.
        /// </summary>
        public float SideFriction { get; set; }

        /// <summary>
        /// Friction of the car in forward direction.
        /// </summary>
        public float ForwardFriction { get; set; }

        /// <summary>
        /// The length of the suspension spring.
        /// </summary>
        public float WheelTravel { get; set; }

        /// <summary>
        /// If set to true the wheel blocks.
        /// </summary>
        public bool Locked { get; set; }

        /// <summary>
        /// The highest possible velocity of the wheel.
        /// </summary>
        public float MaximumAngularVelocity { get; set; }

        /// <summary>
        /// The number of rays used for this wheel.
        /// </summary>
        public int NumberOfRays { get; set; }

        /// <summary>
        /// The position of the wheel in body space.
        /// </summary>
        public System.Numerics.Vector3 Position { get; set; }

        /// <summary>
        /// Creates a new instance of the Wheel class.
        /// </summary>
        /// <param name="world">The world.</param>
        /// <param name="car">The RigidBody on which to apply the wheel forces.</param>
        /// <param name="position">The position of the wheel on the body (in body space).</param>
        /// <param name="radius">The wheel radius.</param>
        public Wheel(World world, RigidBody car, System.Numerics.Vector3 position, float radius)
        {
            this.world = world;
            this.car = car;
            this.Position = position;

            raycast = new RaycastCallback(RaycastCallback);

            // set some default values.
            this.SideFriction = 1.5f;
            this.ForwardFriction = 1f;
            this.Radius = radius;
            this.Inertia = 1.0f;
            this.WheelTravel = 0.2f;
            this.MaximumAngularVelocity = 200;
            this.NumberOfRays = 5;
        }

        /// <summary>
        /// Gets the position of the wheel in world space.
        /// </summary>
        /// <returns>The position of the wheel in world space.</returns>
        public System.Numerics.Vector3 GetWorldPosition()
        {
            return car.Position +
                JMath.Transform(Position + JMath.Up * displacement, car.Orientation);
        }

        /// <summary>
        /// Adds drivetorque.
        /// </summary>
        /// <param name="torque">The amount of torque applied to this wheel.</param>
        public void AddTorque(float torque)
        {
            driveTorque += torque;
        }

        public void PostStep(float timeStep)
        {
            if (timeStep <= 0.0f) return;

            float origAngVel = angVel;
            upSpeed = (displacement - lastDisplacement) / System.Math.Max(timeStep, JMath.Epsilon);

            if (Locked)
            {
                angVel = 0;
                torque = 0;
            }
            else
            {
                angVel += torque * timeStep / Inertia;
                torque = 0;

                if (!lastOnFloor) driveTorque *= 0.1f;

                // prevent friction from reversing dir - todo do this better
                // by limiting the torque
                if (((origAngVel > angVelForGrip) && (angVel < angVelForGrip)) ||
                     ((origAngVel < angVelForGrip) && (angVel > angVelForGrip)))
                    angVel = angVelForGrip;

                angVel += driveTorque * timeStep / Inertia;
                driveTorque = 0;

                float maxAngVel = this.MaximumAngularVelocity;
                angVel = JMath.Clamp(angVel, -maxAngVel, maxAngVel);

                WheelRotation += (timeStep * angVel) / (2 * JMath.Pi) * 360.0f;
            }
        }

        public void PreStep(float timeStep)
        {
            float vel = car.LinearVelocity.Length();

            SideFriction = 2.5f - JMath.Clamp(vel / 20.0f, 0.0f, 1.4f);
            ForwardFriction = 5.5f - JMath.Clamp(vel / 20.0f, 0.0f, 5.4f);

            System.Numerics.Vector3 force = System.Numerics.Vector3.Zero;

            System.Numerics.Vector3 worldAxis = JMath.Transform(JMath.Up, car.Orientation);
            System.Numerics.Vector3 worldPos = car.Position + JMath.Transform(Position, car.Orientation);

            System.Numerics.Vector3 forward = new System.Numerics.Vector3(-car.Orientation.M31, -car.Orientation.M32, -car.Orientation.M33);

            System.Numerics.Vector3 wheelFwd = JMath.Transform(forward, System.Numerics.Matrix4x4.CreateFromAxisAngle(JMath.Up, SteerAngle / 360 * 2 * JMath.Pi));
            System.Numerics.Vector3 wheelLeft = System.Numerics.Vector3.Cross(JMath.Up, wheelFwd); wheelLeft = System.Numerics.Vector3.Normalize(wheelLeft);
            System.Numerics.Vector3 wheelUp = System.Numerics.Vector3.Cross(wheelFwd, wheelLeft);

            float rayLen = 2.0f * Radius + WheelTravel;

            System.Numerics.Vector3 wheelRayStart = worldPos;
            System.Numerics.Vector3 wheelDelta = -Radius * worldAxis;
            System.Numerics.Vector3 wheelRayEnd = worldPos + wheelDelta;

            float deltaFwd = (2.0f * Radius) / (NumberOfRays + 1);
            float deltaFwdStart = deltaFwd;

            lastDisplacement = displacement;
            displacement = 0.0f;

            lastOnFloor = false;

            System.Numerics.Vector3 rayOrigin = car.Position + JMath.Transform(Position, car.Orientation);

            System.Numerics.Vector3 groundNormal = System.Numerics.Vector3.Zero;
            System.Numerics.Vector3 groundPos = System.Numerics.Vector3.Zero;
            float deepestFrac = float.MaxValue;
            RigidBody worldBody = null;

            for (int i = 0; i < NumberOfRays; i++)
            {
                float distFwd = (deltaFwdStart + i * deltaFwd) - Radius;
                float zOffset = Radius * (1.0f - (float)Math.Cos(Math.PI / 4 * (distFwd / Radius)));

                System.Numerics.Vector3 newOrigin = wheelRayStart + distFwd * wheelFwd + zOffset * wheelUp;

                RigidBody body; System.Numerics.Vector3 normal; float frac;
                bool result = world.CollisionSystem.Raycast(newOrigin, wheelDelta,
                    raycast, out body, out normal, out frac);



                if (result && frac <= 1.0f)
                {
                    if (frac < deepestFrac)
                    {
                        deepestFrac = frac;
                        groundPos = rayOrigin + frac * wheelDelta;
                        worldBody = body;
                        groundNormal = normal;
                    }

                    lastOnFloor = true;
                }
            }

            if (!lastOnFloor) return;

            if (groundNormal.LengthSquared() > 0.0f) groundNormal = System.Numerics.Vector3.Normalize(groundNormal);

            // System.Diagnostics.Debug.WriteLine(groundPos.ToString());


            displacement = rayLen * (1.0f - deepestFrac);
            displacement = JMath.Clamp(displacement, 0.0f, WheelTravel);

            float displacementForceMag = displacement * Spring;

            // reduce force when suspension is par to ground
            displacementForceMag *= Math.Abs(System.Numerics.Vector3.Dot(groundNormal, worldAxis));

            // apply damping
            float dampingForceMag = upSpeed * Damping;

            float totalForceMag = displacementForceMag + dampingForceMag;

            if (totalForceMag < 0.0f) totalForceMag = 0.0f;

            System.Numerics.Vector3 extraForce = totalForceMag * worldAxis;

            force += extraForce;

            System.Numerics.Vector3 groundUp = groundNormal;
            System.Numerics.Vector3 groundLeft = System.Numerics.Vector3.Cross(groundNormal, wheelFwd);
            if (groundLeft.LengthSquared() > 0.0f) groundLeft = System.Numerics.Vector3.Normalize(groundLeft);

            System.Numerics.Vector3 groundFwd = System.Numerics.Vector3.Cross(groundLeft, groundUp);

            System.Numerics.Vector3 wheelPointVel = car.LinearVelocity +
                    System.Numerics.Vector3.Cross(car.AngularVelocity, JMath.Transform(Position, car.Orientation));

            // rimVel=(wxr)*v
            System.Numerics.Vector3 rimVel = angVel * System.Numerics.Vector3.Cross(wheelLeft, groundPos - worldPos);
            wheelPointVel += rimVel;

            System.Numerics.Vector3 worldVel = worldBody.LinearVelocity +
             System.Numerics.Vector3.Cross(worldBody.AngularVelocity, groundPos - worldBody.Position);

            wheelPointVel -= worldVel;

            // sideways forces
            float noslipVel = 0.1f;
            float slipVel = 0.1f;
            float slipFactor = 0.7f;

            float smallVel = 3;
            float friction = SideFriction;

            float sideVel = System.Numerics.Vector3.Dot(wheelPointVel, groundLeft);

            if ((sideVel > slipVel) || (sideVel < -slipVel))
                friction *= slipFactor;
            else
                if ((sideVel > noslipVel) || (sideVel < -noslipVel))
                friction *= 1.0f - (1.0f - slipFactor) * (System.Math.Abs(sideVel) - noslipVel) / (slipVel - noslipVel);

            if (sideVel < 0.0f)
                friction *= -1.0f;

            if (System.Math.Abs(sideVel) < smallVel)
                friction *= System.Math.Abs(sideVel) / smallVel;

            float sideForce = -friction * totalForceMag;

            extraForce = sideForce * groundLeft;
            force += extraForce;

            // fwd/back forces
            friction = ForwardFriction;
            float fwdVel = System.Numerics.Vector3.Dot(wheelPointVel, groundFwd);

            if ((fwdVel > slipVel) || (fwdVel < -slipVel))
                friction *= slipFactor;
            else
                if ((fwdVel > noslipVel) || (fwdVel < -noslipVel))
                friction *= 1.0f - (1.0f - slipFactor) * (System.Math.Abs(fwdVel) - noslipVel) / (slipVel - noslipVel);

            if (fwdVel < 0.0f)
                friction *= -1.0f;

            if (System.Math.Abs(fwdVel) < smallVel)
                friction *= System.Math.Abs(fwdVel) / smallVel;

            float fwdForce = -friction * totalForceMag;

            extraForce = fwdForce * groundFwd;
            force += extraForce;

            // fwd force also spins the wheel
            System.Numerics.Vector3 wheelCentreVel = car.LinearVelocity +
             System.Numerics.Vector3.Cross(car.AngularVelocity, JMath.Transform(Position, car.Orientation));

            angVelForGrip = System.Numerics.Vector3.Dot(wheelCentreVel, groundFwd) / Radius;
            torque += -fwdForce * Radius;

            // add force to car
            car.AddForce(force, groundPos + 0.5f * JMath.Up);

            // add force to the world
            if (!worldBody.IsStatic)
            {
                worldBody.AddForce(force * (-1) * 0.01f, groundPos);
            }

        }

        private bool RaycastCallback(RigidBody body, System.Numerics.Vector3 normal, float frac)
        {
            return (body != car);
        }

    }
}
