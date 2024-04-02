// Copyright (c) LukeLib

package frc.robot.extras.interpolators;

public class BilinearInterpolator {

    private final double[][] lookupTable;
    private final double dataRowSize;
  
    /**
     * Handles finding values through a lookup table in a linear fashion.
     * @param lookupTable an array containing {x_input, y_input, x_val, y_val} points. The collected data must be in ascending y order, collected from left to right, reading up.
     */
    public BilinearInterpolator(double[][] lookupTable) {
        this.lookupTable = lookupTable;
        // If you don't use rows of seven, replace the numbers 6, 7, 8 in the commands with the dataRowSize variable with dataRowSize - 1, dataRowSize, dataRowSize + 1
        //TODO: POST Comp Incorporate this variable
        this.dataRowSize = lookupTable[0].length;
    }

    /**
     * Returns an interpolated array from the lookup table corresponding to the given input x and y value.
     */
    public double[] getLookupValue(double inputXValue, double inputYValue) {   
        //if the note center is approximately in the middle, then only need to do y linear interpolation (no bilinear :)
        if (inputXValue >= 155 && inputXValue<= 165) {
            for (int i = 0; i < lookupTable.length-8; i += 7) {
                // check if it is equal to a collected data point
                if (inputYValue == lookupTable[i][1]) {
                    return new double[]{lookupTable[i][2], lookupTable[i][3]};
                }
        
                //interpolate in purely y-direction
                else if (inputYValue < lookupTable[i][1] && inputYValue > lookupTable[i+7][1]){
                    // if x matches up with a data pt, interpolate between above's data pts' y's to determine pose
                    double ySlope = (lookupTable[i + 7][3] - lookupTable[i][3]) / (lookupTable[i + 7][1] - lookupTable[i][1]);
                    double yYIntercept = lookupTable[i][3];
                    return new double[]{lookupTable[i][2], ySlope * (inputYValue - lookupTable[i][1]) + yYIntercept};
                }
            }
        //in case nothing works or like the note x is good but its too far away in y direction (test)
        return new double[]{0.0,-1.0};
        }

        //if the note center is less than half the x_pixels (160)
        if (inputXValue < 160) {
            //if note center is less than halfway, then we reflect across y-axis
            inputXValue = 320 - inputXValue;
            //Loop through the second row to the second to top most row drawing diagonal vectors. Cannot start at top/bottom because there cannot be diagonal vectors in both directions
            for (int i = 7; i < lookupTable.length-7; i++) {
                //check if it is equivalent to collected data point
                if (inputXValue == lookupTable[i][0]) {
                    if (inputYValue == lookupTable[i][1]) {
                        return new double[]{-(lookupTable[i][2]), lookupTable[i][3]};
                    }
                }
                
                //bilinear interpolation approximation used diagnoal vectored pt (facing top right)
                if (inputXValue > lookupTable[i][0] && inputYValue < lookupTable[i][1] && inputXValue < lookupTable[i+8][0] 
                && inputYValue > lookupTable[i+8][1]) {
                    //Interpolate diagonal points
                    double xSlope = (lookupTable[i + 8][2] - lookupTable[i][2]) / (lookupTable[i + 8][0] - lookupTable[i][0]);
                    double xYIntercept = lookupTable[i][2];
                    double ySlope = (lookupTable[i + 8][3] - lookupTable[i][3]) / (lookupTable[i + 8][1] - lookupTable[i][1]);
                    double yYIntercept = lookupTable[i][3];
        
                    return new double[]{-(xSlope * (inputXValue - lookupTable[i][0]) + xYIntercept), 
                    ySlope * (inputYValue - lookupTable[i][1]) + yYIntercept};
                }

                //bilinear interpolation approximation used diagnoal in opposite direction vectored pt (facing bottom right)
                if (inputXValue > lookupTable[i][0] && inputYValue > lookupTable[i][1] && inputXValue < lookupTable[i-6][0] 
                && inputYValue < lookupTable[i-6][1]) {
                    //Interpolate diagonal points
                    double xSlope = (lookupTable[i - 6][2] - lookupTable[i][2]) / (lookupTable[i - 6][0] - lookupTable[i][0]);
                    double xYIntercept = lookupTable[i][2];
                    double ySlope = (lookupTable[i - 6][3] - lookupTable[i][3]) / (lookupTable[i - 6][1] - lookupTable[i][1]);
                    double yYIntercept = lookupTable[i][3];
        
                    return new double[]{-(xSlope * (inputXValue - lookupTable[i][0]) + xYIntercept), 
                    ySlope * (inputYValue - lookupTable[i][1]) + yYIntercept};
                }
            }
        
            //check for first row for only x interpolation
            for (int i = 0; i < 7; i++) {
              if (inputYValue  == lookupTable[i][1]) {
                if (inputXValue == lookupTable[i][0]) {
                    return new double[]{-lookupTable[i][2], lookupTable[i][3]};
                }
                else {
                // if y matches up with a data pt, interpolate between adjacent (right for over 160) data pts' x's to determine pose
                    double xSlope = (lookupTable[i + 1][2] - lookupTable[i][2]) / (lookupTable[i + 1][0] - lookupTable[i][0]);
                    double xYIntercept = lookupTable[i][2];
                    return new double[]{-(xSlope * (inputYValue - lookupTable[i][0]) + xYIntercept), lookupTable[i][3]};
                }
              }
            }    
        
            //check for last row just to check for x only interpolation or point checking. (You can't do y_interpolation or diagonal)
            for (int i = lookupTable.length-7; i < lookupTable.length; i++) {
              if (inputYValue  == lookupTable[i][1]) {
                if (inputXValue == lookupTable[i][0]) {
                    return new double[]{-lookupTable[i][2], lookupTable[i][3]};
                }
                else {
                    // if y matches up with a data pt, interpolate between adjacent (right for over 160) data pts' x's to determine pose
                    double xSlope = (lookupTable[i + 1][2] - lookupTable[i][2]) / (lookupTable[i + 1][0] - lookupTable[i][0]);
                    double xYIntercept = lookupTable[i][2];
                    return new double[]{-(xSlope * (inputYValue - lookupTable[i][0]) + xYIntercept), lookupTable[i][3]};
                }
              }
            }
        
            //if outside the collected data pt range, 
            //then return unused values for now. I could make a handle to get the max one and then 
            //interpolate or get max for both if its outside and thats what the above 4 cases are supposed to do but the 
            //logic may need to be changed
            return new double[]{-1.0,-1.0, 69.0};
          }

          
        //if the note center is greater than half the x_pixels (160)
        if (inputXValue > 160) {
            //Loop through the second row to the second to top most row drawing diagonal vectors. Cannot start at top/bottom because there cannot be diagonal vectors in both directions
            for (int i = 7; i < lookupTable.length-7; i++) {
                //check to see if the found note center is equivalent to a collected data point
                if (inputXValue == lookupTable[i][0]) {
                    if (inputYValue == lookupTable[i][1]) {
                        return new double[]{lookupTable[i][2], lookupTable[i][3]};
                    }
                }

                //bilinear interpolation approximation used diagnoal vectored pt (facing top right)
                if (inputXValue > lookupTable[i][0] && inputYValue < lookupTable[i][1] && inputXValue < lookupTable[i+8][0] 
                && inputYValue > lookupTable[i+8][1]) {
                    //interpolate in both directions using vector
                    double xSlope = (lookupTable[i + 8][2] - lookupTable[i][2]) / (lookupTable[i + 8][0] - lookupTable[i][0]);
                    double xYIntercept = lookupTable[i][2];
                    double ySlope = (lookupTable[i + 8][3] - lookupTable[i][3]) / (lookupTable[i + 8][1] - lookupTable[i][1]);
                    double yYIntercept = lookupTable[i][3];

                    return new double[]{xSlope * (inputXValue - lookupTable[i][0]) + xYIntercept, 
                        ySlope * (inputYValue - lookupTable[i][1]) + yYIntercept};
                }
    
                //bilinear interpolation approximation used diagnoal in opposite direction vectored pt
                if (inputXValue > lookupTable[i][0] && inputYValue > lookupTable[i][1] && inputXValue < lookupTable[i-6][0] 
                && inputYValue < lookupTable[i-6][1]) {
                    //interpolate in both directions using vector
                    double xSlope = (lookupTable[i - 6][2] - lookupTable[i][2]) / (lookupTable[i - 6][0] - lookupTable[i][0]);
                    double xYIntercept = lookupTable[i][2];
                    double ySlope = (lookupTable[i - 6][3] - lookupTable[i][3]) / (lookupTable[i - 6][1] - lookupTable[i][1]);
                    double yYIntercept = lookupTable[i][3];

                    return new double[]{xSlope * (inputXValue - lookupTable[i][0]) + xYIntercept, 
                        ySlope * (inputYValue - lookupTable[i][1]) + yYIntercept};
                }
            }
        
            //check for first row for only x interpolation
            for (int i = 0; i < 7; i++) {
                if (inputYValue  == lookupTable[i][1]) {
                    if (inputXValue == lookupTable[i][0]) {
                        return new double[]{lookupTable[i][2], lookupTable[i][3]};
                    }
                    else {
                        // if y matches up with a data pt, interpolate between adjacent (right for over 160) data pts' x's to determine pose
                        double xSlope = (lookupTable[i + 1][2] - lookupTable[i][2]) / (lookupTable[i + 1][0] - lookupTable[i][0]);
                        double xYIntercept = lookupTable[i][2];
                        return new double[]{xSlope * (inputYValue - lookupTable[i][0]) + xYIntercept, lookupTable[i][3]};
                    }
                }
            }    
        
            //check for last row just to check for x only interpolation or point checking. (You can't do y_interpolation or diagonal)
            for (int i = lookupTable.length-7; i < lookupTable.length; i++) { 
                if (inputYValue  == lookupTable[i][1]) {
                    if (inputXValue == lookupTable[i][0]) {
                        return new double[]{lookupTable[i][2], lookupTable[i][3]};
                    }
                    else {
                    // if y matches up with a data pt, interpolate between adjacent (right for over 160) data pts' x's to determine pose
                        double xSlope = (lookupTable[i + 1][2] - lookupTable[i][2]) / (lookupTable[i + 1][0] - lookupTable[i][0]);
                        double xYIntercept = lookupTable[i][2];
                        return new double[]{xSlope * (inputYValue - lookupTable[i][0]) + xYIntercept, lookupTable[i][3]};
                    }
                }
            }
        
            //if outside the collected data pt range, 
            //then return unused values for now. I could make a handle to get the max one and then 
            //interpolate or get max for both if its outside and thats what the above 4 cases are supposed to do but the 
            //logic may need to be changed
            return new double[]{-1.0,-1.0, 96.0};
        }

    //The third number in each double is just for debugging purposes to determine which case was selected
    return new double[]{-1.0,-1.0, 77.0};

    }
}