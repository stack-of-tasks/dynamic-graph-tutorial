/*
 *  Copyright 2010 CNRS
 *
 *  Florent Lamiraux
 */

#ifndef DG_TUTORIAL_COMMAND_SET_CART_MASS
#define DG_TUTORIAL_COMMAND_SET_CART_MASS

#include <dynamic-graph/command.h>
#include <dynamic-graph/value.h>

namespace dynamicgraph {
  namespace tutorial {
    namespace command {
      using dynamicgraph::command::Command;
      using dynamicgraph::command::Value;
      class SetCartMass : public command::Command {
      public:
	SetCartMass(InvertedPendulum& ip) : Command(ip, typeVector()) {}

	virtual ~SetCartMass() {}
	virtual Value doExecute ()
	{
	  double mass=0;
	  const std::vector<Value>& values = getParameterValues();
	  std::cout << "SetCartMass::doExecute()" << std::endl;
	  for (unsigned int i=0; i<values.size(); i++) {
	    std::cout << "  value[" << i << "]=(" 
		      << values[i] << ")" << std::endl;
	  }
	  
	  mass = values[0].doubleValue();
	  std::cout << "SetCartMass( " << mass << " )" << std::endl;
	  InvertedPendulum& ip = static_cast<InvertedPendulum&>(owner());
	  ip.setCartMass(mass);
	  return Value();
	}

      private:
	static const std::vector<Value::Type> typeVector() 
	{
	  std::vector<Value::Type> result;
	  result.push_back(Value::DOUBLE);
	  return result;
	}
      };
    } // namespace command
  } // namespace tutorial
} // namespace dynamicgraph
#endif
