/* Run with Playwright installed and Edge available. Optional output directory. */
const {chromium}=require('playwright');
const fs=require('fs'),path=require('path'),assert=require('assert');
const {pathToFileURL}=require('url');
(async()=>{
 const out=path.resolve(process.argv[2]||'outputs/kitten-v4');fs.mkdirSync(out,{recursive:true});
 const browser=await chromium.launch({headless:true,channel:'msedge'});
 const page=await browser.newPage({viewport:{width:1440,height:1100},deviceScaleFactor:1});
 const errors=[],requests=[];page.on('pageerror',e=>errors.push(String(e)));page.on('request',r=>requests.push(r.url()));
 await page.goto(pathToFileURL(path.join(__dirname,'index.html')).href);
 await page.waitForFunction(()=>window.DoroDebug?.ready);await page.evaluate(()=>DoroDebug.pause());
 const frames=JSON.parse(fs.readFileSync(path.join(__dirname,'browser_samples.json'),'utf8'));
 let maxError=0;
 for(const sample of frames){
  const points=await page.evaluate(s=>{DoroDebug.setMode(s.mode);DoroDebug.seek(s.time);return Object.fromEntries(Object.keys(s.points).map(n=>[n,DoroDebug.point(n)]));},sample);
  assert(await page.evaluate(()=>DoroDebug.surfacesFinite()));
  for(const [name,p] of Object.entries(points))maxError=Math.max(maxError,Math.hypot(...p.map((x,i)=>x-sample.points[name][i])));
 }
 assert(maxError<1e-9);assert.equal(await page.locator('#jointSelect option').count(),19);
 for(const mode of ['stand','arms','quad']){
  await page.locator(`[data-mode="${mode}"]`).click();
  await page.evaluate(mode=>DoroDebug.seek(mode==='arms'?1.5:0),mode);
  await page.waitForTimeout(250);await page.screenshot({path:path.join(out,mode+'.png'),fullPage:true});
 }
 for(const layer of ['xray','skeleton','skin']){await page.locator(`[data-layer="${layer}"]`).click();assert.equal(await page.locator(`[data-layer="${layer}"]`).getAttribute('aria-pressed'),'true');}
 await page.locator('[data-mode="stand"]').click();await page.locator('[data-layer="skeleton"]').click();
 await page.waitForTimeout(150);await page.screenshot({path:path.join(out,'joints.png'),fullPage:true});
 await page.selectOption('#jointSelect','rear_L_knee');await page.locator('#jointRange').fill('60');
 let state=await page.evaluate(()=>DoroDebug.state());assert(state.manual);assert(Math.abs(state.q[state.joints.indexOf('rear_L_knee')]-Math.PI/3)<1e-9);
 assert((await page.locator('#validation').innerText()).includes('검사하지'));
 await page.locator('[data-mode="arms"]').click();await page.locator('#timeline').fill('500');
 assert.equal((await page.evaluate(()=>DoroDebug.state())).time,3);
 await page.selectOption('#speed','1.5');await page.locator('#play').click();await page.waitForTimeout(200);assert((await page.evaluate(()=>DoroDebug.state())).time>3);
 await page.evaluate(()=>DoroDebug.pause());
 for(const view of ['front','side','three'])await page.locator(`[data-view="${view}"]`).click();
 for(const width of [390,320]){await page.setViewportSize({width,height:900});await page.locator('[data-layer="skin"]').click();await page.locator('[data-mode="stand"]').click();await page.waitForTimeout(200);assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth));await page.screenshot({path:path.join(out,`mobile-${width}.png`),fullPage:true});}
 for(const href of await page.locator('a[href]').evaluateAll(a=>a.map(x=>x.getAttribute('href'))))assert(fs.existsSync(path.resolve(__dirname,href)),'Missing link '+href);
 assert(!requests.some(u=>/^https?:/.test(u)),'Viewer should be offline');assert.deepEqual(errors,[]);
 const report={browser:'Microsoft Edge via Playwright',viewports:[1440,390,320],joint_options:19,fk_sample_poses:frames.length,max_browser_mujoco_link_position_error_m:maxError,javascript_errors:errors,external_requests:0,controls_checked:['four mode initialization paths','layers','joint selection and manual angle','timeline','playback','speed','camera view buttons'],limitations:['no dynamic balance test','manual angles not collision validated']};
 fs.writeFileSync(path.join(out,'browser-report.json'),JSON.stringify(report,null,2)+'\n');console.log(JSON.stringify(report,null,2));await browser.close();
})().catch(e=>{console.error(e);process.exit(1)});
