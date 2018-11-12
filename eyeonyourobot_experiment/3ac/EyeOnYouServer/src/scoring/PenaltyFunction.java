package scoring;

/**
* This class is used to calculate penalties with user-specified parameters.
*
* @author  WeiChun
*/
public class PenaltyFunction {

	private double Lambda_1;
	private double Alpha;
	
	/**
	* construct a penalty function
	*   
	* @param: lambda
	* @param: alpha
	*/
	public PenaltyFunction(double lambdaValue, double AlphaValue)
	{
		Lambda_1 = lambdaValue;
		Alpha = AlphaValue;
	}

	/**
	* output penalty score
	*   
	* @param: duraion of NCO in 0.1-second
	* 
	* @return: penalty score
	*/
	public double getPenaltyValue(double duration)
	{
		return -(Lambda_1*Math.exp(Alpha*duration));
	}
	
}