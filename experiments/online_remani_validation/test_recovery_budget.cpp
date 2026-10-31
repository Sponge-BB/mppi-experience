#include "recovery_budget.hpp"
#include <cassert>
#include <iostream>
int main(){
  recovery_budget::Window budget(10.,.6);
  assert(budget.canStart(10.49));assert(budget.claim(10.49));
  assert(!budget.canStart(10.50));assert(!budget.claim(10.50));
  assert(!budget.recoveryExpired(10.55));assert(budget.recoveryExpired(10.57));
  assert(!budget.expired(10.59));assert(budget.expired(10.61));
  assert(budget.expired(9.99));assert(budget.expired(NAN));
  recovery_budget::Window late(10.,.6);
  assert(!late.claim(10.55));assert(!late.claim(10.61));assert(!late.claimed());
  assert(late.remaining(10.55)<.06);
  bool invalid=false;try{recovery_budget::Window bad(0.,-.1);}catch(const std::invalid_argument&){invalid=true;}
  assert(invalid);
  std::cout<<"RECOVERY_BUDGET_PASSED one claim; original absolute deadline; validation reserve; no clock reset\n";
}
