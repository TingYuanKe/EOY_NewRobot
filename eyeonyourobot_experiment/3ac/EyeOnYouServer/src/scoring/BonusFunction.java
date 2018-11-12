package scoring;

/**
* This class is used to calculate bonuses with user-specified parameters.
*
* @author  WeiChun
*/
public class BonusFunction {
	
	private double Lambda_2;
	private double Beta;
	
	/**
	* construct a bonus function
	*   
	* @param: lambda
	* @param: beta
	*/
	public BonusFunction(double lambdaValue, double BetaValue)
	{
		Lambda_2 = lambdaValue;
		Beta = BetaValue;
	}

	/**
	* output bonus score
	*   
	* @param: duraion of PCO in 0.1-second
	* 
	* @return: bonus score
	*/
	public double getBonusValue(double duration)
	{
		//return Lambda_2*Math.exp(-(duration*duration/Beta));
		return Lambda_2*(1/(1+Math.exp(-(duration-Beta))));
	}
	
}