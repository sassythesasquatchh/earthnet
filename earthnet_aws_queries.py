import boto3
import awswrangler as wr
import pandas as pd

DATABASE_NAME = "weatherStation"
TABLE_NAME = "weatherStation"

filename = 'earthnet_database.csv'


SELECT_ALL = f""" SELECT * FROM {DATABASE_NAME}.{TABLE_NAME} """

# QUERY_TEMP = f""" SELECT Device, CREATE_TIME_SERIES(time, measure_value::varchar) as temp 
#     FROM {DATABASE_NAME}.{TABLE_NAME} 
# 	WHERE measure_name= 'temp'
#     group by Device """
   

QUERY_TEMP = f""" SELECT Device, time, measure_value::varchar 
    FROM {DATABASE_NAME}.{TABLE_NAME} 
	WHERE measure_name= 'temp'
    group by Device, time, measure_value::varchar """


QUERY_HUMIDITY = f""" SELECT Device, CREATE_TIME_SERIES(time, measure_value::varchar) as humidity 
    FROM {DATABASE_NAME}.{TABLE_NAME} 
	WHERE measure_name= 'humidity'
    group by Device """

QUERY_PM25 = f""" SELECT Device, CREATE_TIME_SERIES(time, measure_value::varchar) as PM25
    FROM {DATABASE_NAME}.{TABLE_NAME} 
	WHERE measure_name= 'PM25'
    group by Device """

QUERY_PM10 = f""" SELECT Device, CREATE_TIME_SERIES(time, measure_value::varchar) as PM10 
    FROM {DATABASE_NAME}.{TABLE_NAME} 
	WHERE measure_name= 'PM10'
    group by Device """

df = wr.timestream.query(SELECT_ALL)
df.to_csv(filename, index=False)
print(df)