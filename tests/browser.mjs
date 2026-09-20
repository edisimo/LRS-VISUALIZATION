import {chromium} from '@playwright/test';
const browser=await chromium.launch({headless:true,args:['--no-sandbox','--use-gl=angle','--use-angle=swiftshader','--enable-webgl']});
const page=await browser.newPage({viewport:{width:1440,height:1000}});const errors=[];await page.route('**/*',route=>{if(new URL(route.request().url()).hostname!=='127.0.0.1'){errors.push('External request: '+route.request().url());return route.abort();}return route.continue();});page.on('pageerror',e=>errors.push(e.message));page.on('console',m=>{if(m.type()==='error')errors.push(m.text());});
const started=Date.now();await page.goto('http://127.0.0.1:8765/#mapping');await page.waitForFunction(()=>window.__lab?.ready,{timeout:20000});
console.log('Ready in',Date.now()-started,'ms (headless Chromium software WebGL)');
const topics=['mapping','planning','trajectory','avoidance','full_pipeline'];
for(const topic of topics){
 await page.locator(`[data-topic="${topic}"]`).click();await page.waitForFunction(topic=>window.__lab?.topic===topic,topic);
 const modes=await page.locator('#demo option').allTextContents();
 for(const mode of modes){await page.selectOption('#demo',mode);await page.locator('#progress').evaluate(e=>{e.value='650';e.dispatchEvent(new Event('input'));});await page.waitForFunction(mode=>window.__lab?.mode===mode&&window.__lab.progress===.65,mode);if(!(await page.locator('#metrics').textContent()).length)throw Error('No metrics: '+mode);console.log('OK',topic,mode);}
 if(topic==='planning'){await page.selectOption('#demo','Informed RRT*');await page.locator('#progress').evaluate(e=>{e.value='950';e.dispatchEvent(new Event('input'));});await page.waitForTimeout(250);await page.screenshot({path:'tests/planning.png'});}
 if(topic==='avoidance'){await page.selectOption('#demo','MPPI');await page.locator('#progress').evaluate(e=>{e.value='500';e.dispatchEvent(new Event('input'));});await page.waitForTimeout(250);await page.screenshot({path:'tests/avoidance.png'});}
}
for(let stage=0;stage<12;stage++){const value=Math.round((stage+.5)/12*1000);await page.locator('#progress').evaluate((e,value)=>{e.value=String(value);e.dispatchEvent(new Event('input'));},value);await page.waitForFunction(stage=>document.querySelectorAll('.stage')[stage].classList.contains('active'),stage);}
await page.screenshot({path:'tests/pipeline.png'});
await page.locator('[data-topic="mapping"]').click();await page.selectOption('#demo','Point cloud');await page.waitForTimeout(250);await page.screenshot({path:'tests/mapping.png'});
await page.selectOption('#demo','Inflation');await page.locator('#param-radius').fill('1.2');await page.locator('#param-margin').fill('0.8');await page.waitForTimeout(250);if(!(await page.locator('#metrics').textContent()).includes('0.00 m'))throw Error('Inflation control did not close aisle');await page.locator('#reset').click();await page.waitForTimeout(200);if(await page.locator('#param-radius').inputValue()!=='0.25')throw Error('Reset failed');await page.locator('#top').click();await page.locator('#play').click();await page.waitForTimeout(500);await page.locator('#play').click();if((await page.evaluate(()=>window.__lab.progress))<=0)throw Error('Playback failed');
if(errors.length)throw Error(errors.join('\n'));await browser.close();console.log('All browser views passed.');
