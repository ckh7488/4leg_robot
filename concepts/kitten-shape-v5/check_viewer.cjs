/* Run with Playwright installed and Edge available. Writes screenshots outside source. */
const {chromium}=require('playwright'),fs=require('fs'),path=require('path'),assert=require('assert'),{pathToFileURL}=require('url');
(async()=>{
 const out=path.resolve(process.argv[2]||'outputs/kitten-v5');fs.mkdirSync(out,{recursive:true});
 const browser=await chromium.launch({headless:true,channel:'msedge'});
 try{
 const page=await browser.newPage({viewport:{width:1280,height:1000},deviceScaleFactor:1}),errors=[],network=[];
 page.on('pageerror',e=>errors.push(String(e)));page.on('request',r=>{if(/^https?:/.test(r.url()))network.push(r.url())});
 await page.goto(pathToFileURL(path.join(__dirname,'index.html')).href);await page.waitForFunction(()=>window.KittenDebug?.ready);await page.evaluate(()=>KittenDebug.pause());
 const modes=require('./pose.js').modes,bounds={};
 assert.equal(await page.locator('[data-mode]').count(),7);
 for(const [mode,m] of Object.entries(modes)){
  await page.locator(`[data-mode="${mode}"]`).click();assert.equal(await page.locator(`[data-mode="${mode}"]`).getAttribute('aria-pressed'),'true');
  for(const fraction of [0,.25,.5,.75,1]){await page.evaluate(t=>KittenDebug.seek(t),m.period*fraction);assert(await page.evaluate(()=>KittenDebug.finite()));}
  await page.evaluate(t=>KittenDebug.seek(t),mode==='paw'?m.period/2:0);
  bounds[mode]=await page.evaluate(()=>KittenDebug.bounds());
  await page.waitForTimeout(80);await page.locator('.viewport').screenshot({path:path.join(out,mode+'.png')});
 }
 for(const layer of ['rig','skin']){await page.locator(`[data-layer="${layer}"]`).click();assert.equal((await page.evaluate(()=>KittenDebug.state())).layer,layer);}
 await page.locator('[data-mode="walk"]').click();await page.locator('#timeline').fill('500');assert(Math.abs((await page.evaluate(()=>KittenDebug.state())).time-.8)<1e-9);
 await page.selectOption('#speed','1.5');await page.locator('#play').click();await page.waitForTimeout(220);assert.notEqual((await page.evaluate(()=>KittenDebug.state())).time,.8);await page.evaluate(()=>KittenDebug.pause());
 for(const view of ['side','front','top','three']){await page.locator(`[data-view="${view}"]`).click();assert.equal(await page.locator(`[data-view="${view}"]`).getAttribute('aria-pressed'),'true');}
 await page.locator('[data-mode="idle"]').click();await page.screenshot({path:path.join(out,'full.png'),fullPage:true});
 for(const width of [390,320]){await page.setViewportSize({width,height:900});await page.waitForTimeout(80);assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth));await page.screenshot({path:path.join(out,'mobile-'+width+'.png'),fullPage:true});}
 for(const href of await page.locator('a[href]').evaluateAll(a=>a.map(x=>x.getAttribute('href'))))assert(fs.existsSync(path.resolve(__dirname,href)),href);
 assert.deepEqual(errors,[]);assert.deepEqual(network,[]);
 const report={browser:'Edge via Playwright',viewports:[1280,390,320],poses:7,surface_finite_samples:35,controls:['pose selection','skin and posing rig','timeline','playback and speed','four camera views'],javascript_errors:errors,external_requests:network,visual_bounds_at_displayed_poses_m:bounds,scope:'Browser behavior and finite display geometry only. Bounding boxes are not collision tests. Naturalness needs user review.'};
 fs.writeFileSync(path.join(__dirname,'browser_validation.json'),JSON.stringify(report,null,2)+'\n');console.log(JSON.stringify(report,null,2));
 }finally{await browser.close();}
})().catch(e=>{console.error(e);process.exit(1)});
