#include "../SmartRegulator/Types.h"
#include "../SmartRegulator/Types.cpp"

#include <cassert>
#include <string.h>

int main(int argc, char *argv[]) {
	struct {
		float value;
		char  precision;
		const char *expected;
	} tests[] = {
		{1.234, 2, "1.23"},
		{-1.234, 1, "-1.2"},
		{0.0, 4, "0.0000"},
		{3.14159265, 6, "3.141592"},
		{1.9999, 2, "1.99"}		// do we want to round?
	};

	// test float formatter
	for ( int t = 0; t < sizeof(tests)/sizeof(tests[0]); t++) {
		assert(strcmp(floatString(tests[t].value, tests[t].precision), tests[t].expected) == 0);
	}

	// we don't return the same pointer over and over again
	// so the data is in a different buffer each time
	assert(floatString(1.1, 1) != floatString(1.1, 1));

	printf("All tests passed.\n");
}
