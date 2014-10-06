/*
 * cmd_terminal.hpp
 *
 *  Created on: Sep 26, 2014
 *      Author: walmis
 */

#ifndef CMD_TERMINAL_HPP_
#define CMD_TERMINAL_HPP_

#include <xpcc/architecture.hpp>
#include <xpcc/io/terminal.hpp>

using namespace xpcc;

class CmdTerminal : public Terminal {
public:
	CmdTerminal(IODevice& device) : Terminal(device), ios(device) {

	};

	xpcc::IOStream ios;
protected:

	void handleCommand(uint8_t nargs, char* argv[]);

};

extern CmdTerminal terminal;


#endif /* CMD_TERMINAL_HPP_ */
