package frc.robot.extras;

public class BilinearInterpolator {

  private final double[][] lookupTable;
  
  /**
   * Handles finding values through a lookup table in a linear fashion.
   * @param lookupTable an array containing {x, y} points, the x values must be in ascending order.
   */
  public BilinearInterpolator(double[][] lookupTable) {
    this.lookupTable = lookupTable;
  }

  /**
   * Returns a linearly-interpolated value from the lookup table corresponding to the given input value.
   */
  public double[] getLookupValue(double inputXValue, double inputYValue) {   
          //if the note center is in the middle, then only need to do y linear interpolation (no bilinear :)
          if (inputXValue >= 155 && inputXValue<= 165) {
            for (int i = 0; i < lookupTable.length-8; i += 7) {
              if (inputYValue == lookupTable[i][1]) {
              return new double[]{lookupTable[i][2], lookupTable[i][3]};
              }
        
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
            // if we haven't reached any cases nor the previous case, we cannot interpolate with diagonally farther vector since there are no more data pts
            // if we haven't reached any cases nor the previous case, we cannot interpolate with diagonally farther vector since there are no more data pts
            inputXValue = 320 - inputXValue;
            for (int i = 7; i < lookupTable.length-7; i++) {
        
              if (inputXValue == lookupTable[i][0]) {
        
                if (inputYValue == lookupTable[i][1]) {
                return new double[]{-(lookupTable[i][2]), lookupTable[i][3]};
                }
        
              }
              
              //bilinear interpolation approximation used diagnoal vectored pt
              if (inputXValue > lookupTable[i][0] && inputYValue < lookupTable[i][1] && inputXValue < lookupTable[i+8][0] 
              && inputYValue > lookupTable[i+8][1]) {
        
                double xSlope = (lookupTable[i + 8][2] - lookupTable[i][2]) / (lookupTable[i + 8][0] - lookupTable[i][0]);
                double xYIntercept = lookupTable[i][2];
        
                double ySlope = (lookupTable[i + 8][3] - lookupTable[i][3]) / (lookupTable[i + 8][1] - lookupTable[i][1]);
                double yYIntercept = lookupTable[i][3];
        
                return new double[]{-(xSlope * (inputXValue - lookupTable[i][0]) + xYIntercept), 
                  ySlope * (inputYValue - lookupTable[i][1]) + yYIntercept};
              }
    
            //bilinear interpolation approximation used diagnoal in opposite direction vectored pt
              if (inputXValue > lookupTable[i][0] && inputYValue > lookupTable[i][1] && inputXValue < lookupTable[i-6][0] 
              && inputYValue < lookupTable[i-6][1]) {
        
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

        //   //if the note center is less than half the x_pixels (160)
        //   if (inputXValue < 160) {
        //     // if we haven't reached any cases nor the previous case, we cannot interpolate with diagonally farther vector since there are no more data pts
        //     for (int i = 7; i < mirrored.length-7; i++) {
        
        //       if (inputXValue == mirrored[i][0]) {
        
        //         if (inputYValue == mirrored[i][1]) {
        //         return new double[]{mirrored[i][2], mirrored[i][3]};
        //         }
        //       }
              
        //       //bilinear interpolation approximation used diagnoal vectored pt
        //       if (inputXValue < mirrored[i][0] && inputYValue < mirrored[i][1] && inputXValue > mirrored[i+8][0] 
        //       && inputYValue > mirrored[i+8][1]) {
        
        //         double xSlope = (mirrored[i + 8][2] - mirrored[i][2]) / (mirrored[i + 8][0] - mirrored[i][0]);
        //         double xYIntercept = mirrored[i][2];
        
        //         double ySlope = (mirrored[i + 8][3] - mirrored[i][3]) / (mirrored[i + 8][1] - mirrored[i][1]);
        //         double yYIntercept = mirrored[i][3];
        
        //         return new double[]{xSlope * (inputXValue - mirrored[i][0]) + xYIntercept, 
        //           ySlope * (inputYValue - mirrored[i][1]) + yYIntercept};
        //       }
    
        //       //bilinear interpolation approximation used diagnoal in opposite direction vectored pt
        //       if (inputXValue < lookupTable[i][0] && inputYValue > lookupTable[i][1] && inputXValue > lookupTable[i-6][0] 
        //       && inputYValue < lookupTable[i-6][1]) {
        
        //         double xSlope = (lookupTable[i - 6][2] - lookupTable[i][2]) / (lookupTable[i - 6][0] - lookupTable[i][0]);
        //         double xYIntercept = lookupTable[i][2];
        
        //         double ySlope = (lookupTable[i - 6][3] - lookupTable[i][3]) / (lookupTable[i - 6][1] - lookupTable[i][1]);
        //         double yYIntercept = lookupTable[i][3];
        
        //         return new double[]{xSlope * (inputXValue - lookupTable[i][0]) + xYIntercept, 
        //           ySlope * (inputYValue - lookupTable[i][1]) + yYIntercept};
        //       }
        //     }
        
        //     //check for first row for only x interpolation
        //     for (int i = 0; i < 7; i++) {
              
        //       if (inputYValue  == lookupTable[i][1]) {
                
        //         if (inputXValue == lookupTable[i][0]) {
        //         return new double[]{lookupTable[i][2], lookupTable[i][3]};
        //         }
        
        //         else {
        //         // if y matches up with a data pt, interpolate between adjacent (right for over 160) data pts' x's to determine pose
        //         double xSlope = (lookupTable[i + 1][2] - lookupTable[i][2]) / (lookupTable[i + 1][0] - lookupTable[i][0]);
        //         double xYIntercept = lookupTable[i][2];
        //         return new double[]{xSlope * (inputYValue - lookupTable[i][0]) + xYIntercept, lookupTable[i][3]};
        //         }
        
        //       }
        //     }   
        
        //     //check for last row just to check for x only interpolation or point checking. (You can't do y_interpolation or diagonal)
        //     for (int i = mirrored.length-7; i < mirrored.length; i++) {
              
        //       if (inputYValue  == mirrored[i][1]) {
                
        //         if (inputXValue == mirrored[i][0]) {
        //         return new double[]{mirrored[i][2], mirrored[i][3]};
        //         }
        
        //         else {
        //         // if y matches up with a data pt, interpolate between adjacent (right for over 160) data pts' x's to determine pose
        //         double xSlope = (mirrored[i + 1][2] - mirrored[i][2]) / (mirrored[i + 1][0] - mirrored[i][0]);
        //         double xYIntercept = mirrored[i][2];
        //         return new double[]{xSlope * (inputYValue - mirrored[i][0]) + xYIntercept, mirrored[i][3]};
        //         }
        
        //       }
        //     }
        
        //     //if outside the collected data pt range, 
        //     //then return unused values for now. I could make a handle to get the max one and then 
        //     //interpolate or get max for both if its outside and thats what the above 4 cases are supposed to do but the 
        //     //logic may need to be changed
        //     return new double[]{-1.0,-1.0, 69.0};
        
        //   }
        
        
        
        
        
        
        
        
        
        
        
          //if the note center is greater than half the x_pixels (160)
          if (inputXValue > 160) {
            // if we haven't reached any cases nor the previous case, we cannot interpolate with diagonally farther vector since there are no more data pts
            for (int i = 7; i < lookupTable.length-7; i++) {
        
              if (inputXValue == lookupTable[i][0]) {
        
                if (inputYValue == lookupTable[i][1]) {
                return new double[]{lookupTable[i][2], lookupTable[i][3]};
                }
        
              }
              
              //bilinear interpolation approximation used diagnoal vectored pt
              if (inputXValue > lookupTable[i][0] && inputYValue < lookupTable[i][1] && inputXValue < lookupTable[i+8][0] 
              && inputYValue > lookupTable[i+8][1]) {
        
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
        
                
        return new double[]{-1.0,-1.0};
        
          }
}