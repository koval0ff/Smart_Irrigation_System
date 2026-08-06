(()=>{
 const root=document.documentElement;
 const switcher=document.querySelector('.language-switcher');
 const buttons=[...document.querySelectorAll('.lang-button')];
 const textNodes=[...document.querySelectorAll('[data-i18n]')];
 function setLanguage(lang){
   if(!window.TRANSLATIONS[lang])lang='en';
   root.lang=lang;
   textNodes.forEach(el=>{const key=el.dataset.i18n;const value=window.TRANSLATIONS[lang][key];if(value)el.textContent=value;});
   buttons.forEach(btn=>{const active=btn.dataset.lang===lang;btn.classList.toggle('active',active);btn.setAttribute('aria-pressed',String(active));});
   switcher.dataset.active=lang;
   document.body.classList.remove('language-fade');void document.body.offsetWidth;document.body.classList.add('language-fade');
   try{localStorage.setItem('sis-language',lang)}catch(e){}
 }
 let initial='en';try{const browser=(navigator.language||'').toLowerCase();initial=localStorage.getItem('sis-language')||(browser.startsWith('de')?'de':browser.startsWith('uk')?'uk':'en')}catch(e){}
 buttons.forEach(btn=>btn.addEventListener('click',()=>setLanguage(btn.dataset.lang)));setLanguage(initial);
 const observer=new IntersectionObserver(entries=>entries.forEach(entry=>{if(entry.isIntersecting){entry.target.classList.add('visible');observer.unobserve(entry.target)}}),{threshold:.12,rootMargin:'0px 0px -35px'});
 document.querySelectorAll('.reveal').forEach(el=>observer.observe(el));
 const soil=document.getElementById('soil-value'),bar=document.getElementById('soil-bar'),pump=document.getElementById('pump-status');
 const states=[{v:3474,w:84,p:'Standby',pd:'Bereit',pu:'Очікування'},{v:3060,w:74,p:'Checking',pd:'Prüfung',pu:'Перевірка'},{v:2890,w:68,p:'Watering',pd:'Bewässerung',pu:'Полив'},{v:2200,w:52,p:'Standby',pd:'Bereit',pu:'Очікування'}];let si=0;
 setInterval(()=>{si=(si+1)%states.length;const s=states[si];soil.textContent=s.v;bar.style.width=s.w+'%';pump.textContent=root.lang==='de'?s.pd:root.lang==='uk'?s.pu:s.p;pump.style.color=s.p==='Watering'?'#34c759':''},2400);
 const dialog=document.getElementById('lightbox'),dialogImg=dialog.querySelector('img');
 document.querySelectorAll('.gallery-item').forEach(btn=>btn.addEventListener('click',()=>{dialogImg.src=btn.dataset.full;dialog.showModal()}));
 dialog.querySelector('button').addEventListener('click',()=>dialog.close());dialog.addEventListener('click',e=>{if(e.target===dialog)dialog.close()});
 document.getElementById('year').textContent=new Date().getFullYear();
})();
