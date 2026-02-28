package com.example;

public class Test {
    public static void main(String[] args) {
        System.out.println(errorIsOscillating(new double[] { 50, 1, 50, 1, 1 }));
    }
    private static double noPowerThreshold = 1, prevEncoderOscillatingSize = 3;
    private static boolean errorIsOscillating(double[] prevErrors) {
        boolean oscillating = true;
        boolean prevInBound = false;
        boolean inBoundEveryTime = true;
        for (int i = 0; i < prevEncoderOscillatingSize; i++) {
            boolean curInBound = Math.abs(prevErrors[i]) <= noPowerThreshold;
            if(!curInBound)
                inBoundEveryTime = false;
            if(i == 0) {
                prevInBound = curInBound;
                continue;
            }
            if(prevInBound == curInBound) {
                oscillating = false;
                break;
            }
            prevInBound = curInBound;
        }
        if(oscillating)
            return true;
        if(inBoundEveryTime)
            return false;
        boolean ranAtLeastOnce = false;
        for(int i = 0; i < prevErrors.length; i++) {
            boolean curInBound = Math.abs(prevErrors[i]) <= noPowerThreshold;
            if(i == 0) {
                prevInBound = curInBound;
                continue;
            }
            if (prevErrors[i] == noPowerThreshold)
                continue;

            ranAtLeastOnce = true;
            if(prevInBound == curInBound)
                return false;
            prevInBound = curInBound;
        }
        if (!ranAtLeastOnce)
            return false;
        return true;
    }
}
