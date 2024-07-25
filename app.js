
const fs = require('fs');

const db = new Map([
   [1,{txt:"Оплата робітникам відряникам за відрядними розцінками та нормами виробітку ",fond:1}],
   [2,{txt:"Оплата робітникам з погодинною формою оплати праці(карта-завдання)",fond:1}],
   [3,{txt:"Оплата за посадовими окладами за відпрацьований час ",fond:1}],
   [4,{txt:"Оплата робітникам з погодинною формою оплати за тарифними ставками та окладами за відпрацьований час",fond:1}],
   [5,{txt:"Оплата робітникам на окладі (шкіливі умови праці)",fond:1}],
   [6,{txt:"Оплата робітникам на тарифній ставці (шкіливі умови праці)",fond:1}],
   [7,{txt:"Оплата робітникам зпогодиною формою оплати праці (оклад,тарифна ставка)",fond:1}],
   [8, {txt:"Доплата за шкідливі умови праці",fond:1}],
   [9, {txt:"Премія робітникам з погодинною формою оплати праці (карта-завдання)",fond:1}],
   [10, {txt:"Премія робітникам з погодинною формою оплатою праці (шньші)",fond:1}],
   [11, {txt:"винагорода за проведення льотних випробувань повітряних суден",fond:1}],
   [12,{txt: "оплата за роботу у вихідні, святкові та неробочі дні (ІТП)",fond:1}],
   [13,{txt: "оплата за вихідні святкові неробочі дні (відрядження)",fond:1}],
   [14,{txt: "оплата за роботу у надурочний час робітникам з погодинною формою оплати",fond:1}],
   [15,{txt: "премія робітникам-відрядникам за виконання виробничих завдань ",fond:1}],
   [16, {txt:"оплата за роботу робітникам-відрядникам у надурочний час",fond:1}],
   [17, {txt:"премія з фонду майстра",fond:1}],
   [18,{txt: "премія за особисте клеймо",fond:1}],
   [19, {txt:"харчування льотного екіпажу і технічної бригади ПС з коефіцієнтом",fond:1}],
   [20,{txt: "оплата по середньому заробітку працівникам за виконання державних та громадських обов'язків",fond:1}],
   [21,{txt:"оплата додоаткових відпусток у зв'язку з навчанням та творчих відпуток",fond:1}],
   [22,{txt:"надбавки до добових за особливі умови праці",fond:1}],
   [23,{txt:"доплата за керівництво бригадою",fond:1}],
   [24,{txt:"доплата за суміщеня профессій",fond:1}],
   [25,{txt: "Доплата підліткам",fond:1}],
   [26,{txt:"Доплата за тимчасове замісництво",fond:1}],
   [27,{txt:"Доплат за роботу в нічний час",fond:1}],
   [28,{txt:"Збережені виплати (доплати) згідно законодавства",fond:1}],
   [29, {txt:"простій з вини працівника",fond:8}],
   [30, {txt:"оплата за роботу у вихідні, святкові та неробочі дні рообітникам з погодинною формою оплати праці",fond:1}],
   [31, {txt:"виплата у натуральній формі із собівартості",fond:2}],
   [32,{txt:"надбавка до окладу молодим працівникам",fond:1}],
   [33, {txt:"надбавка за класність водіям",fond:1}],
   [34,{txt: "оплата щорічних (основної та додаткової) відпусток поточного місяця",fond:1}],
   [36, {txt:"оплата у дні закордонних відряджень (довгострокові)",fond:1}],
   //------------------------
   [37, {txt:"оплата праці під час перебування у відрядженні",fond:1}],
   [38, {txt:"оплата по середньому заробітку часу навчання і підвищення кваліфікації",fond:1}],
   [39, {txt:"відпустка для догляду за дитиною до досягнення нею віку 3х років",fond:7}],
   [40, {txt:"доплата водіям за ненормований робочий день",fond:1}],
   [41, {txt:"доплата водіям за миття автомобіля",fond:1}],
   [42, {txt:"оплатаводіям на ремонті",fond:1}],
   [44, {txt:"премія за економію матеріальних трудових енергоресурсів",fond:1}],
   [45, {txt:"одноразові заохочення що не порв'язані з конкретними результатами праці Ст.24",fond:1}],
   [46, {txt:"виплата грошової допомоги (6 посадових оклдів) при виході на наукову пенсію",fond:4}],
   [47, {txt:"доплата за виплату грошей (касирам)",fond:1}],
   [48, {txt:"оплата внутрішнього сумісництва",fond:1}],
   [49, {txt:"компенація за невикористану відпустку при звільненні",fond:1}],
   [50, {txt:"Винагорода за вислугу років",fond:1}],
   [51, {txt:"допомога по тимчасовій непрацездатності за дні минулого місяця",fond:7}],
   [52,  {txt:"допомога по тимчасовій непрацездатності за дні поточного місяця",fond:7}],
   [53, {txt:"неоплачувана допомога по  тимчасовій непрацездатності",fond:7}],
   [54, {txt:"допомога по вагітності та пологах миннуого місяця",fond:7}],
   [55, {txt:"допомога по вагітності та пологах поточного місяця",fond:7}],
   [56, {txt:"допомога по вагітності та пологах наступного місяця",fond:7}],
   [57, {txt:"одноразова матеріальна допомога не оподаткована ПДФО",fond:2}],
   [58, {txt:"цільова благодійна допомога підприємства працівнику",fond:2}],
   [59, {txt:"нарахування за актами перевірки",fond:4}],
   [60, {txt:"виплати за спеціальними виплатами адміністрації та профкому",fond:2}],
   [61, {txt:"виплати у вигляді відшкодування матеріального та морального збитку ",fond:2}],
   [62, {txt:"премія за виконання тематчних планів",fond:1}],
   [63, {txt:"індексація зарплати",fond:1}],
   [64, {txt:"оплата перекладачам - внутрішнім сумісникам (відрядно)",fond:1}],
   [65, {txt:"винагорода за високий професіоналізм та якість роботи",fond:1}],
   [66, {txt:"премія за виконання найважливійших планових завдань",fond:7}],
   [67, {txt:"допомога по тимчасовій непрацездатності по догляду за дитиною (минулий місяць)",fond:7}],
   [68,{txt:"допомога по тимчасовій непрацездатності по догляду за дитиною (поточний місяць)",fond:7}],
   [69,{txt:"допомога по тимчасовій непрацездатності по догляду за дитиною (наступний місяць)",fond:7}],
   [70, {txt:"оплата відпусток за навчання у розмірі 50% із ФЗП",fond:1}],
   [71, {txt:"доплата за науковий ступінь",fond:1}],
   [72, {txt:"доплата до середнього заробітку при переведенні на легку працю",fond:1}],
   [73, {txt:"оплата учням, перекваліфікантам за час навчання",fond:1}],
   [74, {txt:"доплата за збільшення обсягу робіт (ІТП)",fond:1}],

   [75, {txt:"грошова компенсація за частину невикистаної відпустки",fond:1}],
   [76, {txt:"доплата за умови праці (понад норми законодавства)",fond:4}],
   [77, {txt:"відшкодування матеріальних цінностей (інструмент)",fond:4}],
   [78, {txt:"борг за підприємством понад один місяць і більше (звільнені)",fond:8}],
   [79, {txt:"одноразові заохоченя жінкам до 8 березня",fond:2}],
   [80, {txt:"премія з нагоди міжнародного жіночого дня 8 березня)",fond:4}],
   [81, {txt:"одноразова матеріальна допомога громадянам ВВВ і громадянам прирівнених до них",fond:2}],
   [82, {txt:"додаткова доплата працівникам спеціалізованої бригади АТ 'АНТОНОВ'",fond:1}],
   [83, {txt:"надбавка в умовах обмеженого режиму",fond:1}],
   [84, {txt:"оплата додаткової відпустки ліквідаторам 1-2 кат із чорнобильського фонду",fond:9}],
   [85, {txt:"доплата до мінімалної заробітної плати",fond:1}],
   [86, {txt:"депонована заробітна плата",fond:8}],
   [87, {txt:"оплата роботи викладачів та керівників виробничою практикою",fond:1}],
   [88, {txt:"оплата часу простою не з вини працівника",fond:1}],
   [89, {txt:"доплата доплата за роботу в святкові та неробочі дні за графіком",fond:1}],
   [90, {txt:"оплата матеріальних і соціальних додаткових благ",fond:2}],
   [91, {txt:"борг за підприємством на початок місяця",fond:8}],
   [92, {txt:"оплата винагороди за транспортні польоти",fond:1}],
   [93, {txt:"доплата за збільшення обсягу робіт (робітникам)",fond:1}],
   [94, {txt:"оплата працівникам-донорам днів обстеження, здавання крові та відпочнку",fond:1}],
   [95, {txt:"оплата перших 5 днів тимчасової непрацездатності за рахунок коштів піприємсва", fond:4}],
   [96, {txt:"одноразова матеріаьна допомога працівникам при виході на пенсію ",fond:2}],
   [97, {txt:"оплата регламентних перерв ", fond:1}],
   [98, {txt:"сума вихідної допомоги при припиненні трудового договору", fond:4}],
   [99, {txt:"прибуття, вибуття (прийом чи звільнення працівника)",fond:10}],
   [100, {txt:"матеріальна допомога за рішенням коміссії ",fond:2}],
   [101, {txt:"надбавка до добових понад встановленого розміру",fond:2}],
   [102, {txt:"допомога затверджена адміністрацією в наказах і положеннях ",fond:2}],
   [103, {txt:"оплата допомоги при звільненні в троьохкратному розмірі  ліквідаторам 1-2 кат",fond:9}],
   ///****************** */
   [104, {txt:"одноразові заохочення особам, які не перебувають у штаті",fond:2}],

   [105, {txt:"надбавка за інтенивність праці та особливий характер роботи ",fond:1}],
   [106, {txt:"оплата державних і гроадських обов'язків з прибутку ",fond:4}],
   [107, {txt:"матеріальна допомога на оздоровлення та іньші соціальні потреби ",fond:4}],

   [109, {txt:"вартість безкоштовно наданго працівникам форменного спецдягу ",fond:4}],
   [111, {txt:"допомога на поховання за рішенням коміссії",fond:2}],
   [112, {txt:"додаткова відпустка жінкам згдно колдогвору",fond:2}],
   [113, {txt:"вартість спецодягу спецвзуття та ЗІЗ що виаюьтся понад норми",fond:2}],
   [114, {txt:"іньш виплати соціального характеру (додаткові блага) ",fond:2}],
   [115, {txt:"допомога з тимчасової непрацездатності з ФСС НВ ",fond:3}],
   [116, {txt:"доплат аза роботу  в шцідливих умовах праці нв нвдурочний час та вихідні",fond:4}],
   [117, {txt:"компенсації працівникам за використання для потреб виробництва власного інструменту та особистого транспорту ",fond:2}],
   [703, {txt:"сума зарплати що перевищує 10 мінімальних зарплат ",fond:1}],
   [710, {txt:"оплата робітникам відрядникам за відрядними розцінкамми та нормами виробітку у відряженні  ",fond:1}],
   [711, {txt:"оплата робтникам  погодинною формою оплати праці (кар-завдвння) у відрядженні",fond:1}],
   [712, {txt:"оплта за посадовими окладами за відпрацьований час у відрядженні ",fond:1}],
   [713, {txt:"оплата робітникам за погодинно формою оплати за трифними ставкамии та окладами за відпрацьований час у відрядженні  ",fond:1}],
   [714, {txt:"оплата робітникам за погодинно формою оплати праці (оклад, тарифна ставка) по табелю (не рахувати у відрядженні) ",fond:1}],
   [721, {txt:"матеріальна допомога (строкова служба) ",fond:2}],
   [729, {txt:"матеріальна допомога  (контракт)",fond:2}],
   [731, {txt:"матеріальна допомога (військова служба по мобілізації) ",fond:2}],
   [737, {txt:"сумма яка перевищує середню за час час перебування у відрядженні ",fond:1}],
   [800, {txt:"доплата згідно спільного рішення",fond:2}],
   [801 , {txt:"доплата міжрозрядної різниці  ",fond:1}],
   [802 , {txt:"доплата мололдим робітникуам за період освоєння профессії",fond:1}],
   [803, {txt:"оплата по середньому заробітку за час перебування в клініці профпатології",fond:1}],
   [804 , {txt:"доплата за робту у вечірній час (режим багатозмінний)  ",fond:1}],

   [806 , {txt:"премія робітникам-відрядникам за виконання виробничих завдань ",fond:1}],
   [807 , {txt:"премія згідно розпорядження суб'єктів управління  ",fond:4}],
   [808, {txt:"винагорода суб'єктів управління (за підсумками року згідно чистого прибутку)  ",fond:2}],
   [809 , {txt:"премія згідно розпорядження суб'єктів управління (за підсумками кварталу за рахунок чистого доходу)",fond:4}],
   [810, {txt:"премія службовцям за виконання планових завдань",fond:1}],
   [811 , {txt:"премія робітникам з прогодинною формою оплати праці за виконання виробничих завдань ",fond:1}],
   [812 , {txt:"виплата соц. характеру у грошовій і натуральній формі (оплата гуртожитку) ",fond:4}],
   [813 , {txt:"виплата соц. характеру у грошовій і натуральній формі (харчування) ",fond:4}],
   [814 , {txt:"оплата за  раціоналізаторські виплати ",fond:1}],
   [815 , {txt:"надбавка до окладу працівникам  ",fond:4}],
   [816 , {txt:"невиплачені лікарнні листки через касу ",fond:7}],

   [817 , {txt:"невиплачені лікарнні листки через ПУМБ  ",fond:7}],
   [818 , {txt:"невиплачені лікарнні листки через ПриватБанк  ",fond:7}],
   [819 , {txt:"невиплачені лікарнні листки через Ощадбанк  ",fond:7}],
   [820 , {txt:"оплата  середнього заробітку мобілізованим працівникам",fond:1}],
   [821 , {txt:"оплата середнього заробітку призваним на строкову військову службу ",fond:1}],
   [822, {txt:"виплата за дні перебування на військових зборах ",fond:1}],
   [823 , {txt:"невиплачена зараплата через Приватбанк",fond:8}],
   [824 , {txt:"невиплачена зараплата через Ощадбанк",fond:8}],
   [825 , {txt:"допомога за умовами праці згідно колективному договору (за рішенням підприємства) ",fond:4}],
   [826, {txt:"невиплачена зарплата через касу  ",fond:2}],
   [827, {txt:"оплата по середньому за час проходження медогляду ",fond:1}],

   [828, {txt:"виплати за рішенням суду  ",fond:2}],
   [829 , {txt:"оплата по середньому призваних працівників на контрактні основі ",fond:1}],
   [830, {txt:"оплата по середньому добровольцям Тероборони  ",fond:1}],
   [831, {txt:"оплата по середньому заробітку призваним на в\службу по мобілізації  ",fond:1}],


   [834 , {txt:"оплата відпустки за рік що передує поточному",fond:1}],
   [835, {txt:"оплата відпустки за майбутній календарний рік ",fond:1}],
   [836, {txt:"відпустка при народженні дитини (татові)  ",fond:1}],
   [837, {txt:"доплата до середньої зарплати за час перебування у відрядженні",fond:1}],
   [838, {txt:"сума судового збоу для повернення ",fond:4}],

   [851 , {txt:"доплата по тимчасовій непрацездатності за дні минулого місяця (звільнені)  ",fond:7}],
   [867 , {txt:"доплата по тимчасовій непрацездатності по догляду зв хворим (звільнені) ",fond:7}],
   [879, {txt:"вартіть подарунків які не включаються до розрахунку загального місячного (річного) оподаткованого доходу ",fond:2}],
   [880, {txt:"одноразові заохочення (подарунки) в матеріальній формі ",fond:2}],
   [881, {txt:"оплата додаткової відпустки учасникам бойових дій або інвалідам війни",fond:1}],
   [882, {txt:"оплата додаткової відпустки працівникам які мають дітей (ст.19 зак про відпустку)",fond:1}],
   [883, {txt:"оплата відпустки працівникам (за минулий період)",fond:1}],
   [884 , {txt:"оплата за роботу у вихідні дні ІТП в одинарному розмірі (відгул)",fond:1}],
   [885 , {txt:"оплата за роботу у вихідні дні робітникам відрядникам в одинарному розмірі (відгул)",fond:1}],
   [886 , {txt:"оплата за роботу у вихідні дні робітникам з погодинною формою плати праці в одинарному розмірі (відгул)",fond:1}],
   [887 , {txt:"доплата керівниам адаптації та наставникам ",fond:1}],
   [888, {txt:"оплата часів простою не  з вини працівника (по середньому зар.)",fond:1}],
   [889 , {txt:"додаткове благо (менеджер) у період відрядження",fond:1}],
   [890 , {txt:"оплата по-середньому за час надзвичайних обставин",fond:1}],
   [891 , {txt:"призупинення дії трудового договору ",fond:8}],
   [895, {txt:"доплата по тимчсовій непрацездатності за перші 5 днів (звільнені) ",fond:2}],
   [896 , {txt:"борг, який утворився на 28.02.2022 на підставі виконання п.2.5.15 кд",fond:8}],
   [900, {txt:"матеріальна допомога згідно спільного рішеня ",fond:1}],
   [901, {txt:"винагорода по ЦПХ",fond:1}],
   [903, {txt:"оплата згідно наказу №165  ДК 'Укроборонпром' ",fond:1}],
   [914 , {txt:"виплати соцального характеру (пільгове харчування робітників дитячих оздоровчих комплексів)  ",fond:4}],
   [920, {txt:"виконання державних та громадських обов'язків без оплати",fond:8}],
   [922 , {txt:"надбавка за умови підвищеної завантаженості льотного екіпажу ",fond:1}],
   [971, {txt:"простій з вини підприємства без оплати ",fond:8}],
   [972, {txt:"адміністративна відпустка за заявою працівника що виїхав за кордон або набув стсусу ВПО(ст.12)",fond:8}],
   [980 , {txt:"компенсація за невикористану відпустку колишньому працівнику підприємства  ",fond:2}],
   [981 , {txt:"відгул за працю у вихідний ",fond:8}],
   [982, {txt:"неповний робочий день ",fond:8}],
   [983 , {txt:"вартість закордонних паспортів оформлеих працівниками згідно рішення адміністрації ",fond:1}],
   [984, {txt:"вартісьт обов'язкового медичного страхування членам льотного екіпажу повітряних суден та технічним бригадам за рішенням адмінітрації ",fond:1}],
   [985 , {txt:"пільга при оплаті за протезування",fond:2}],
   [986 , {txt:"борг за працівником по ФСС",fond:7}],
   [987 , {txt:"проводка за депонентів по ФСС з ТВП ",fond:7}],
   [988 , {txt:"проводка за депонентів по ФСС з НВ ",fond:7}],
   [989 , {txt:"проводка  по ФСС ",fond:8}],
   [990 , {txt:"проводка  по ФСС НВ ",fond:8}],
   [991 , {txt:"адміністративна відпустка за згодою сторін на період карантину (ч з ст26)",fond:8}],
   [992 , {txt:"адміністративна відпустка по нагляду за дитиною від 3 до 6 років",fond:8}],
   [993 , {txt:"прогул",fond:8}],
   [994 , {txt:"запізненя",fond:8}],
   [995 , {txt:"зпесвоєчасний вихід з роботи",fond:8}],
   [996 , {txt:"записка про звільнення",fond:8}],
   [997 , {txt:"адміністративна відпустка по наказу (ст. 25 крім п.3 Закону України)",fond:8}],
   [998 , {txt:"адміністративна відпустка по наказу (ст. 26   Закону України)",fond:8}],
   [999 , {txt:"невідоме",fond:8}],

 ])


let outputArrayIdx="";
let outputArrayDescription="";
let outputArrayFond="";

outputArrayFond +="const short Fond[]={";
outputArrayDescription += " const wchar_t * Description[]={";
for (let iteration=0; iteration<1024; iteration++){
 let item = db.get(iteration);
 if(item === undefined){
    outputArrayDescription +="";
    outputArrayFond +=0;
 }
}



