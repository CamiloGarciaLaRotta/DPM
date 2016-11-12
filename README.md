# DPM
**API Documentation**
https://camilogarcialarotta.github.io/DPM/index.html

**GOLDEN RULES OF VERSION CONTROL**   
1. Do not talk about fight club.  
2. Keep the code, doc and commits clean and concise.  
3. Never commit to master, one branch per feature.  
4. Test before you push.  
5. Test after you merge.  

**JULIETTE'S GUIDE TO WRITE TESTABLE CODE**   
1. Every public function should do one task.  
2. If you write a function to fetch sensor data you should use that function everywhere.   
3. Make it modular. Removing one task shouldn't break anything else.  
4. Avoid race conditions. it's easy to make a damn mutex and hard to debug race conditions.  
5. Don't write measurements in line make a constant for it.  
6. Functions that just process data just use the data as an input, not what gets the data. Unit tests are fun, mocking hardware isn't.  
7. Seriously though avoid side effects when you can.

