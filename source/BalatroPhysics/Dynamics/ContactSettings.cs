/* Copyright (C) <2009-2011> <Thorben Linneweber, Jitter Physics>
* 
*  This software is provided 'as-is', without any express or implied
*  warranty.  In no event will the authors be held liable for any damages
*  arising from the use of this software.
*
*  Permission is granted to anyone to use this software for any purpose,
*  including commercial applications, and to alter it and redistribute it
*  freely, subject to the following restrictions:
*
*  1. The origin of this software must not be misrepresented; you must not
*      claim that you wrote the original software. If you use this software
*      in a product, an acknowledgment in the product documentation would be
*      appreciated but is not required.
*  2. Altered source versions must be plainly marked as such, and must not be
*      misrepresented as being the original software.
*  3. This notice may not be removed or altered from any source distribution. 
*/

using System;

namespace BalatroPhysics.Dynamics
{
    public struct ContactSettings
    {
        public readonly static ContactSettings Default;
        public enum MaterialCoefficientMixingType { TakeMaximum, TakeMinimum, UseAverage }

        public float MaximumBias { get; set; }

        public float BiasFactor { get; set; }

        public float MinimumVelocity { get; set; }

        public float AllowedPenetration { get; set; }

        public float BreakThreshold { get; set; }

        public MaterialCoefficientMixingType MaterialCoefficientMixing { get; set; }

        static ContactSettings()
        {
            Default = new ContactSettings(10f);
        }

        public ContactSettings(float maximumBias = 10f, float bias = 0.25f, float minVelocity = 0.001f, float allowedPenetration = 0.01f, float breakThreshold = 0.01f, MaterialCoefficientMixingType  materialMode = MaterialCoefficientMixingType.UseAverage)
        {
            MaximumBias = maximumBias;
            BiasFactor = bias;
            MinimumVelocity = minVelocity;
            AllowedPenetration = allowedPenetration;
            BreakThreshold = breakThreshold;
            MaterialCoefficientMixing = materialMode;
        }
    }
}
