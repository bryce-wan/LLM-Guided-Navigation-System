# planner
planner for LLM-guided navigation
### Notice: something important
 ```
pygccxml==2.2.1 is necessary

when you encounter this error:
 - raise runtime_errors.declaration_not_found_t(decl_matcher)
   pygccxml.declarations.runtime_errors.declaration_not_found_t: Unable to find declaration. Matcher: [(decl type==class_t) and (name==SpecificParam< std::string >)]
   
 check the right version above with pygxxcml and pip install castxml
 (when you finished above do not forget rm -rf build or rm -f CMakeCache.txt in build/Release, and rebuild it again)


Strange problem may caused by version differences:
we encountered the following  error when we test the ompl under  python 3.8.19:
 - TypeError: No registered converter was able to produce a C++ rvalue of type bool from this Python object of type numpy.bool_

 then we change the python version to 3.8.18, everything worked fine.
 ```