﻿namespace MathFunctions
{

    public class MFuncts
    {
        public int constrain(int amt, int low, int high)
        {
            if (amt < low)
                return low;
            else if (amt > high)
                return high;
            else
                return amt;
        }
    }
}