/*
 * Beaglebone Open-Source Machine Drive
 * C Source template
 * KTH Project Work - 2018 
 */


// Additional comments:
// -Naming: 	use underscores
// 		limit abbreviations if possible
// 		use lower case names for all occurences of function names and variables,
// 			 fully capitalize macros and use capital letters in struct names
// 		avoid magic variables i.e. don't randomly add 3 somewhere without context,
// 			 create a named variable with the value 3, obvious exceptions apply
// 			 for example incrementing 
// -Formatting: try to keep to the formatting style of these templates for consistency
// -Commenting: Do it


/* ================================== INCLUDES ============================== */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <template.h>


/* ================================== MACROS ================================ */

#define MY_MACRO 5			// Defines my macro, use full capital letters in names


/* ================================== TYPEDEFS ============================== */

/* 
 * Comment describing typedef and why it's needed
 */
typedef struct Roman_Legions {
	char unit_name[50];
	int num_of_units;
} Legion;


/* ================================== FUNCTION PROTOTYPES =================== */

/*
 * Short description of function goes here
 * ARG: list of arguments and what they do
 */ 
static int is_legion_full(Legion legio);


/* ================================== INTERNAL GLOBALS ====================== */

/*
 * Use static variables and functions if they are only used in this source file
 */
static const int legion_unit_max = 5200;

/*
 * Short description of variable, with meaning of values included e.g.:
 * Var = 1 - Rec needed
 * Var = "Else" - No rec needed
 */
static int recruitment_needed = -1;


/* ================================== FUNCTION DEFINITIONS ================== */

int is_legion_full(Legion legio)
{
	// Handle all possible cases!
	if (legio.num_of_units < legion_unit_max) {
	        return 0;
	} else if (legio.num_of_units == legion_unit_max) {
	        return 1;
	} else {
		return -1;
	}
}

int main(void)
{
	// Declare all variables in the beginning of the function 
	Legion legio1;			// Declare Legion 1 variable

	// Specify variables
	legio1.num_of_units = 5180;
	strcpy(legio1.unit_name, "BeagleLegion");
	
	// Handle all possible cases, even if seemingly impossible to occur
	// e.g. the default case below!
	switch(is_legion_full(legio1)) {
		case 1 :
			printf("Ready to serve\n");
			break;
		case 0 :
			printf("Notify the quartermaster to begin recruitment\n");
			recruitment_needed = 1;
			break;
		case -1 :
			printf("Take booze away from quartermaster\n");
			break;
		default :
			printf("What?\n");
	}

	// If legion is not at full capacity, recruit a member
	if (recruitment_needed == 1) {
		while (legio1.num_of_units < 5200) {
			recruit_legionnaire(&legio1.num_of_units);
		}
			printf("Legionnaires recruited, units in legion: %d\n", legio1.num_of_units);
	}

	return 0;
}
