[![Typing SVG](https://readme-typing-svg.demolab.com?font=IBM+Plex+Mono&weight=600&size=25&duration=3000&pause=1000&color=1ACAAA&background=11167B00&multiline=true&width=465&height=65&lines=%F0%9F%91%8B+%D0%9F%D1%80%D0%B8%D0%B2%D0%B5%D1%82%2C+%D0%BC%D1%8B+%D0%BA%D0%BE%D0%BC%D0%B0%D0%BD%D0%B4%D0%B0+Cyber+AI+drone;%D0%B8+%D0%BC%D1%8B+%D1%83%D1%87%D0%B0%D1%81%D1%82%D0%BD%D0%B8%D0%BA%D0%B8+Skills+Camp+2024)](https://git.io/typing-svg)
## О команде
В нашей команде 4 человека:

**Хамматов Булат​**: разрабатывает движение за объектом

![Описание изображения](images/Bulat.jpg)

**Иваненко Артём**: занимаеться созданием и настройкой нейронных сетей

![Описание изображения](images/Artem.jpg)

**Волкова Юлиана​**: разрабатывает движение за объектом

![Описание изображения](images/Yliana.jpg)

**Мухина Кира**: занимается созданием программ для детекции объектов

![Описание изображения](images/Kira.jpeg)

Хотим представить решение поставленной задачи в компетенции "Искусвенный интелект в комплексных беспилотных системах"
## Цель и актуальность работы
Целью нашей роботы стала разработка программного обеспечения для обнаружения Ровера, нарушевшего правила дорожного движения. Разработка программного обеспечения для обнаружения сигнала светофора и стоп-линии.

Использование технологий искусственного интеллекта позволяет не только улучшить точность идентификации объектов, но и способствует адаптации к изменениям окружающей среды и постоянному самосовершенствованию системы за счет обучения на собранных данных.
## Миссия 
Взлететь с точки “H” 
* Обнаружить Ровер 🚘, светофор 🚦 и СТОП линию 🛑.  
* При обнаружении Роверв цвет светодиодной ленты поменять на Сrimson (Ровер движется по траектории: прямоугольник), при обнаружении светофора моргнуть Orange цветом, определять объекты с помощью компьютерного зрения 
* В топике подписать Rover, обозначить цвет. 
* При обнаружении Стоп линии моргнуть pink цветом. 
* После определения всех объектов на поле начать алгоритм слежения. Обнаружить Ровер, который нарушил ПДД, проехал СТОП - линию на красный сигнал светофора. Начать погоню за ровером, при этом должна выводиться фраза “ПРИЖМИТЕСЬ К ОБОЧИНЕ”, способ оповещения выбирается участниками. 
* В топике подписать Rover, обозначить цвет ровера. Цвет светодиодной ленты – режим погоня 
* Следовать за ровером – нарушителем не менее 20 секунд, при следовании цвет светодиодной ленты соответствует режиму погоня 
* Совершить точную посадку на взлетную станцию  
* Задание для ровера: 
* Выехать с зоны парковки. Вкл. фары. 
* Начать алгоритм движения по траектории с нарушением ПДД
  
![Описание изображения](images/mission.png)
## План работ
* Продумать алгоритм работы ​
* Сделать нейронную сеть​
* Написать код распознавания объектов​
* Написать программу следования за объектом​
## Нейронная сеть
Она была создана при помощи ylov8 и roboflow. При помощи roboflow мы создали dataset и загрузили его в ultralytics hub.

![Описание изображения](images/neiro.png)
## Выполнение миссии
Для выполнении миссии нам понадобилась библиотека opencv. Мы использовали различные маски для детекции объектов для последующего использования, чтобы найти объекты. Был создан алгарит для проследующего движения за ровером.

Хотим представить решение поставленной задачи в компетенции "Искусвенный интелект в комплексных беспилотных системах".
## 📖 содержание
* создание [Dataset](https://github.com/Artem1557/hackathon_skills_camp_2024/blob/main/Cyber%20AI%20drone/Data_Cyber_AI_drone.zip)
* обучение [нейронной сети](https://github.com/Artem1557/hackathon_skills_camp_2024/blob/main/Cyber%20AI%20drone/Model_Cyber_AI_drone.pt)
* тестирование [модели](https://github.com/Artem1557/hackathon_skills_camp_2024/blob/main/Cyber%20AI%20drone/Test_Cyber_AI_drone.py)
* результат тестирования: [зеленый свет]([hhttps://github.com/Artem1557/hackathon_skills_camp_2024/blob/main/Cyber%20AI%20drone/Result_Cyber_AI_drone.jpg](https://github.com/Artem1557/hackathon_skills_camp_2024/blob/main/Cyber%20AI%20drone/programm_Cyber_AI_drone.py)), [красный свет](https://github.com/Artem1557/hackathon_skills_camp_2024/blob/main/Cyber%20AI%20drone/Result_Cyber_AI_drone_1.jpg)
* [программа](https://github.com/Artem1557/hackathon_skills_camp_2024/blob/main/Cyber%20AI%20drone/programm_Cyber_AI_drone.py) распознования нарушителя
* [видео](https://github.com/Artem1557/hackathon_skills_camp_2024/blob/main/Cyber%20AI%20drone/video_Cyber_AI_drone.mp4) результат проделанной работы


